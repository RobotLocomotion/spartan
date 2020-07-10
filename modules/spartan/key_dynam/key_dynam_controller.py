from __future__ import print_function

import os
from enum import Enum
import numpy as np
import time
import pygame
import cv2
from collections import OrderedDict
import argparse

# ROS
import rospy
import std_srvs.srv
from rospy_message_converter import message_converter

# spartan
from spartan.utils import utils as spartan_utils
from spartan.utils.control_utils import make_cartesian_gains_msg
from spartan.key_dynam.teleop_mouse_manager import TeleopMouseManager
import spartan.utils.utils as spartan_utils
from spartan.key_dynam.data_capture import DataCapture
from spartan.key_dynam.planar_teleop import PlanarMouseTeleop
from spartan.key_dynam.zmq_utils import ZMQClient
from spartan.key_dynam.planar_teleop import MouseTeleopState
from spartan.key_dynam.software_safety import SoftwareSafety


class KeyDynamControllerState(Enum):
    STOPPED = 0
    DEMONSTRATION_QUEUED = 1
    DEMONSTRATION_IN_PROGRESS = 2


class KeyDynamController(PlanarMouseTeleop):

    def __init__(self, config, use_zmq=True):
        super(KeyDynamController, self).__init__()
        self._mouse_manager.release_mouse_focus()
        self._data_capture = DataCapture()
        self._data_capture.wait_for_initial_messages()
        self._cache = dict()
        self._config = config
        self._control_rate = config['control_rate']
        self._software_safety = SoftwareSafety(tf_buffer=self._data_capture._tf_buffer)
        self._controller_state = KeyDynamControllerState.STOPPED

        if use_zmq:
            self._zmq_client = ZMQClient()

    def main_loop(self):
        """
        Single pass through main loop
        :return:
        :rtype:
        """
        events = self._mouse_manager.get_events()

        # can only do one of these

        if pygame.locals.K_s in events["keydown"]:
            print("keydown: s")
            self.start_teleop()
        elif pygame.locals.K_t in events["keydown"]:
            print("keydown:t")
            self.stop_teleop()
        elif pygame.locals.K_f in events["keydown"]:
            print("keydown:f")
            print("\n\n---START DEMONSTRATION---")
            self.start_demonstration()
        elif pygame.locals.K_g in events["keydown"]:
            print("keydown:f")
            print("\n\n---STOP DEMONSTRATION---")
            self.stop_demonstration()
        elif pygame.locals.K_r in events['keydown']:
            print("keydown: r")
            print("\n\n-----RESETTING------")
            self.reset(key='r')
        elif pygame.locals.K_e in events['keydown']:
            print("keydown: e")
            print('\n\n-----RESETTING----')
            self.reset(key='e')
        elif pygame.locals.K_w in events['keydown']:
            print("keydown: w")
            print('\n\n-----RESETTING----')
            self.reset(key='w')
        elif pygame.locals.K_d in events['keydown']:
            print("keydown: d")
            self.debug_action()
        elif pygame.locals.K_e in events['keydown']:
            ### SEND SINGLE FRAME PLAN
            self.send_single_frame_plan()
        elif pygame.locals.K_h in events['keydown']:
            ## store start state
            print("\n\n-----captured START STATE data-----")
            data = self.collect_latest_data()
            self._cache['start_data'] = data
        elif pygame.locals.K_j in events['keydown']:
            print("\n\n----captured GOAL STATE data-----")
            data = self.collect_latest_data()
            self._cache['goal_data'] = data
        elif pygame.locals.K_k in events['keydown']:
            print("\n\n------saving data to disk----")
            compute_control_action_msg = {'type': "COMPUTE_CONTROL_ACTION",
                                          'data': self._cache['start_data']}

            save_dir = os.path.join("/home/manuelli/data/key_dynam/sandbox",
                                    spartan_utils.get_current_YYYY_MM_DD_hh_mm_ss())

            if not os.path.exists(save_dir):
                os.makedirs(save_dir)

            spartan_utils.save_pickle(compute_control_action_msg,
                                      os.path.join(save_dir, 'compute_control_action_msg.p'))

            plan_msg = {'type': "PLAN",
                        'data': [self._cache['goal_data']],
                        }
            spartan_utils.save_pickle(plan_msg, os.path.join(save_dir, 'plan_msg.p'))
        elif pygame.locals.K_p in events['keydown']:
            print("\n\n----sending saved plan message----\n\n")
            plan_msg_file = "/home/manuelli/data/key_dynam/sandbox/2020-07-07-20-09-54_push_right_box_horizontal/plan_msg.p"
            plan_msg = spartan_utils.load_pickle(plan_msg_file)
            self.send_single_frame_plan(msg=plan_msg)
        elif pygame.locals.K_o in events['keydown']:
            print("\n\n----Executing Open Loop Plan-----")

            # debugging
            # compute_control_action_msg_file = "/home/manuelli/data/key_dynam/sandbox/2020-07-07-20-09-54_push_right_box_horizontal/compute_control_action_msg.p"
            # compute_control_action_msg = spartan_utils.load_pickle(compute_control_action_msg_file)
            # self.execute_plan_open_loop(msg=compute_control_action_msg)

            self.execute_plan_open_loop(msg=None)

        # if it's running, then send out the message
        if self._state == MouseTeleopState.RUNNING:
            msg = self.get_task_space_streaming_message(events)
            self._task_space_streaming.publish(msg)

        # only start the demonstration when we start moving the mouse
        if self._controller_state == KeyDynamControllerState.DEMONSTRATION_QUEUED:
            vx = msg.setpoint_linear_velocity.x
            vy = msg.setpoint_linear_velocity.y
            tol = 1e-4
            if np.linalg.norm(np.array([vx,vy])) > tol:
                self._controller_state = KeyDynamControllerState.DEMONSTRATION_IN_PROGRESS
                print("DEMONSTRATION_IN_PROGRESS")

        if self._controller_state == KeyDynamControllerState.DEMONSTRATION_IN_PROGRESS:
            data = self.collect_latest_data()
            self._cache['plan_data'].append(data)

    def send_zmq_reset(self):
        # send the reset message
        msg_reset = {'type': "RESET"}
        self._zmq_client.send_data(msg_reset)
        self._zmq_client.recv_data()

    def send_single_frame_plan(self, msg=None):
        """
        Capture data at single frame, send it over as a plan
        :return:
        """

        self.send_zmq_reset()

        print("\n---SENDING PLAN----")
        if msg is None:
            plan_data = []
            data = self.collect_latest_data()
            plan_data.append(data)

            msg = {'type': 'PLAN',
                   'data': plan_data,
                   }

        self._zmq_client.send_data(msg)
        resp = self._zmq_client.recv_data()
        print("received response")
        print("resp\n", resp)
        print("\n")

    def collect_latest_data(self):
        ### Collects the latest data

        data = self._data_capture.get_latest_data()
        ee_setpoint_msg = self._task_space_streaming.get_last_published_message()
        if ee_setpoint_msg is None:
            ee_setpoint_msg = self.make_empty_cartesian_goal_point_msg()

        ee_setpoint_data = message_converter.convert_ros_message_to_dictionary(ee_setpoint_msg)
        data['actions'] = {'ee_setpoint': ee_setpoint_data}
        data['action'] = data['actions']  # for compatibility w/ key_dynam

        return data

    def send_control_action_req(self):
        """
        Sends the COMPUT_CONTROL_ACTION message request
        :return:
        """

        print("\n---COMPUTE_CONTROL_ACTION REQUEST----")

        data = self.collect_latest_data()

        msg = {'type': 'COMPUTE_CONTROL_ACTION',
               'data': data,
               }

        self._zmq_client.send_data(msg)
        resp = self._zmq_client.recv_data()

    def execute_plan_open_loop(self,
                               msg=None,  # for debugging, optionally send a message that we already have somehow
                               ):
        print("\n---COMPUTE_CONTROL_ACTION REQUEST----")

        if msg is None:
            data = self.collect_latest_data()
            msg = {'type': 'COMPUTE_CONTROL_ACTION',
                   'data': data,
                   }

        start_time = time.time()
        self._zmq_client.send_data(msg)
        resp = self._zmq_client.recv_data()

        resp_data = resp['data']
        elapsed = time.time() - start_time
        self._mouse_manager.release_mouse_focus()
        print("computing plan took %.3f seconds" % (elapsed))
        print("resp_data.keys()", resp_data.keys())
        print("action_seq\n", resp_data['action_seq'])

        s = raw_input("\nexecute plan? y/n\n")
        if s != "y":
            print("aborting")
            return
        else:
            print("executing plan")

        rate = rospy.Rate(self._control_rate)

        msg = self.make_empty_cartesian_goal_point_msg()
        action_seq = resp_data['action_seq']
        self._task_space_streaming.start_streaming()
        try:
            start_time = time.time()
            prev_time = None
            for i in range(action_seq.shape[0]):
                msg.setpoint_linear_velocity.x = action_seq[i][0]
                msg.setpoint_linear_velocity.y = action_seq[i][1]
                if not self._software_safety.check_message(msg):
                    print("unsafe control, breaking off")
                    self._task_space_streaming.stop_streaming()

                self._task_space_streaming.publish(msg)
                if prev_time is not None:
                    print("elapsed since last message", time.time() - prev_time)
                prev_time = time.time()
                rate.sleep()

            elapsed = time.time() - start_time
            print("plan_duration", elapsed)
            msg_zero = self.make_empty_cartesian_goal_point_msg()
            self._task_space_streaming.publish(msg_zero)
            self._task_space_streaming.publish(msg_zero)
        finally:
            self._task_space_streaming.stop_streaming()

        print("plan finished")

    def test_control_rate(self):

        # send the plan msg
        plan_msg_file = "/home/manuelli/data/key_dynam/sandbox/2020-07-07-20-09-54_push_right_box_horizontal/plan_msg.p"
        plan_msg = spartan_utils.load_pickle(plan_msg_file)
        self.send_single_frame_plan(msg=plan_msg)

        print("\n---COMPUTE_CONTROL_ACTION REQUEST----")

        data = self.collect_latest_data()
        msg = {'type': 'COMPUTE_CONTROL_ACTION',
               'data': data,
               'debug': 1,  # False gets converted to a string, maybe by our convert function
               }
        start_time = time.time()
        self._zmq_client.send_data(msg)
        resp = self._zmq_client.recv_data()
        # this the

        resp_data = resp['data']
        elapsed = time.time() - start_time
        self._mouse_manager.release_mouse_focus()
        print("computing plan took %.3f seconds" % (elapsed))
        print("resp_data.keys()", resp_data.keys())
        print("action_seq\n", resp_data['action_seq'])

        rate = rospy.Rate(self._control_rate)
        msg = self.make_empty_cartesian_goal_point_msg()
        action_seq = resp_data['action_seq']

        prev_time = time.time()
        while True:
            data = self.collect_latest_data()
            msg = {'type': 'COMPUTE_CONTROL_ACTION',
                   'data': data,
                   'debug': 0,
                   }
            self._zmq_client.send_data(msg)
            resp = self._zmq_client.recv_data()
            print("elapsed: %.3f" % (time.time() - prev_time))
            prev_time = time.time()
            rate.sleep()

    def execute_plan_closed_loop(self):

        print("\n---EXECUTING CLOSED LOOP PLAN----")

        data = self.collect_latest_data()
        msg = {'type': 'COMPUTE_CONTROL_ACTION',
               'data': data,
               'debug': 1,  # send number instead
               }
        start_time = time.time()
        self._zmq_client.send_data(msg)
        resp = self._zmq_client.recv_data()
        resp_data = resp['data']
        elapsed = time.time() - start_time
        self._mouse_manager.release_mouse_focus()

        print("computing plan took %.3f seconds" % (elapsed))
        print("resp_data.keys()", resp_data.keys())
        print("action_seq\n", resp_data['action_seq'])

        s = raw_input("\nexecute plan? y/n\n")
        if s != "y":
            print("aborting")
            return
        else:
            print("executing plan")

        self._task_space_streaming.start_streaming()

        rate = rospy.Rate(self._control_rate)
        control_msg = self.make_empty_cartesian_goal_point_msg()
        action_seq = resp_data['action_seq']

        elapsed_time_dict = OrderedDict()
        prev_time = time.time()
        counter = 0
        while True:
            counter += 1
            print("\n\n---- Control Step %d ----" % (counter))
            time1 = time.time()
            data = self.collect_latest_data()
            elapsed_time_dict['collect_data'] = time.time() - time1

            time2 = time.time()
            msg = {'type': 'COMPUTE_CONTROL_ACTION',
                   'data': data,
                   'debug': 0,
                   }
            self._zmq_client.send_data(msg)
            resp = self._zmq_client.recv_data()
            elapsed_time_dict['compute_control_action'] = time.time() - time2

            if resp['type'] != "CONTROL_ACTION":
                print("got message of type %s, stopping streaming" % (resp['type']))
                self.send_zero_velocity_message()
                break

            # extract the control action
            action = resp['data']['action']
            x = print("action", action)

            control_msg.setpoint_linear_velocity.x = action[0]
            control_msg.setpoint_linear_velocity.y = action[1]

            # check if it's a safe action to send
            if not self._software_safety.check_message(control_msg):
                print("unsafe control, breaking off")
                self.send_zero_velocity_message()
                break

            self._task_space_streaming.publish(control_msg)
            elapsed_time_dict['total'] = time.time() - time1
            elapsed_time_dict['time_since_prev'] = time.time() - prev_time

            # print("elapsed_times:\n", elapsed_time_dict)
            print("time:", elapsed_time_dict['total'])
            if False:
                for key, val in elapsed_time_dict.iteritems():
                    print("%s:%.3f" % (key, val))
            prev_time = time.time()
            rate.sleep()

        self._task_space_streaming.stop_streaming()

    def reset_to_start_position(self, ):
        """
        - First moves the robot to a safe position above the tabl
        - Next moves robot to starting position
        :return:
        """
        self.stop_teleop()
        # pose = self._stored_poses_dict["key_dynam"]["above_table_ready_pose"]
        # success = self._robotService.moveToJointPosition(pose, maxJointDegreesPerSecond=30, timeout=5)

        pose = self._stored_poses_dict['debug']['key_dynam_start_pose_open_loop_plan']
        success = self._robotService.moveToJointPosition(pose, maxJointDegreesPerSecond=30, timeout=5)

    def run(self):
        """
        Runs main loop at given speed, 5 Hz
        :return:
        :rtype:
        """
        try:
            rate = rospy.Rate(self._control_rate)
            while not rospy.is_shutdown():
                self.main_loop()
                rate.sleep()
        finally:
            controller._mouse_manager.release_mouse_focus()

    def send_zero_velocity_message(self):
        msg = self.make_empty_cartesian_goal_point_msg()
        self._task_space_streaming.publish(msg)

    def start_demonstration(self):
        """
        Starts the key_dynam demonstration
        :return: 
        """
        # clear the cache
        self._cache = {'plan_data': [],  # list of keyframes for plan (at 5Hz)
                       }
        self._controller_state = KeyDynamControllerState.DEMONSTRATION_QUEUED

    def stop_demonstration(self):
        """
        Ends the key_dynam demonstration
        - stops teleop
        - save data to disk
        :return: 
        """
        self._controller_state = KeyDynamControllerState.STOPPED
        self.send_zero_velocity_message()
        self.stop_teleop()
        self.save_plan_data(self._cache['plan_data'])

    def save_plan_data(self,
                       plan_data,  # list of dicts
                       save_dir=None,
                       camera_name="d415_01",
                       fps=5
                       ):
        # make video of trajectory
        # save dir

        print("plan length", len(self._cache['plan_data']))

        data = {'plan_data': plan_data}
        msg = {'type': "PLAN",
               'data': data}

        if save_dir is None:
            save_dir = os.path.join(spartan_utils.get_data_dir(),
                                    "key_dynam/hardware_experiments/demonstrations/sandbox",
                                    spartan_utils.get_current_YYYY_MM_DD_hh_mm_ss())

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        # make a video
        video_name = os.path.join(save_dir, "video.avi")
        video = None
        for data in plan_data:
            frame = data['observations']['images'][camera_name]['rgb'][:, :]
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if video is None:
                height, width, layers = frame.shape
                video = cv2.VideoWriter(video_name, 0, fps, (width, height))

            video.write(frame)

        cv2.destroyAllWindows()
        video.release()

        # save the plan message
        spartan_utils.save_pickle(msg, os.path.join(save_dir, "plan_msg.p"))
        print("saved plan information at:", save_dir)


def debug(controller):
    plan_msg_file = "/home/manuelli/data/key_dynam/sandbox/2020-07-07-20-09-54_push_right_box_horizontal/plan_msg.p"
    plan_msg = spartan_utils.load_pickle(plan_msg_file)
    controller.send_single_frame_plan(msg=plan_msg)

    compute_control_action_msg_file = "/home/manuelli/data/key_dynam/sandbox/2020-07-07-20-09-54_push_right_box_horizontal/compute_control_action_msg.p"
    compute_control_action_msg = spartan_utils.load_pickle(compute_control_action_msg_file)
    controller.execute_plan_open_loop(msg=compute_control_action_msg)


def test_closed_loop_control(controller,
                             move_to_start_position=True):
    plan_msg_file = "/home/manuelli/data/key_dynam/sandbox/2020-07-07-20-09-54_push_right_box_horizontal/plan_msg.p"
    plan_msg = spartan_utils.load_pickle(plan_msg_file)
    controller.send_single_frame_plan(msg=plan_msg)

    if move_to_start_position:
        print("moving to reset position")
        controller.reset_to_start_position()
        raw_input("press Enter to continue")

    controller.execute_plan_closed_loop()


def test_closed_loop_trajectory_control(controller,
                                        move_to_start_position=True):

    if move_to_start_position:
        print("moving to reset position")
        controller.reset_to_start_position()
        raw_input("press Enter to continue")

    # folder = "2020-07-09-23-59-56_push_long"
    # folder = "2020-07-10-00-01-58_push_short_fast"
    # folder = "2020-07-10-00-15-41_box_vertical"
    folder = "2020-07-10-00-42-58_long_push_top_towards_right"
    plan_msg_file = "/home/manuelli/data/key_dynam/hardware_experiments/demonstrations/stable/%s/plan_msg.p" %(folder)
    plan_msg = spartan_utils.load_pickle(plan_msg_file)
    controller.send_single_frame_plan(msg=plan_msg)



    controller.execute_plan_closed_loop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--teleop", action='store_true', default=False, help="indicates you want to teleop")
    args = parser.parse_args()


    rospy.init_node('key_dynam_controller', anonymous=True)
    config_file = os.path.join(spartan_utils.getSpartanSourceDir(), 'modules/spartan/key_dynam/controller_config.yaml')
    config = spartan_utils.getDictFromYamlFilename(config_file)
    controller = KeyDynamController(config)

    try:
        # debug(controller)

        if args.teleop:
            controller.reset_to_start_position()
            controller._mouse_manager.grab_mouse_focus()
            controller.run()
        else:
            controller._mouse_manager.release_mouse_focus()
            test_closed_loop_trajectory_control(controller,
                                                move_to_start_position=True)
    finally:
        controller._mouse_manager.release_mouse_focus()

    print("Finished cleanly")
