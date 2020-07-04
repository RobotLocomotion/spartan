from __future__ import print_function

import os
from enum import Enum
import time
import pygame


# ROS
import rospy
import std_srvs.srv
from rospy_message_converter import message_converter


# spartan
from spartan.utils.control_utils import make_cartesian_gains_msg
from spartan.key_dynam.teleop_mouse_manager import TeleopMouseManager
import spartan.utils.utils as spartan_utils
from spartan.key_dynam.data_capture import DataCapture
from spartan.key_dynam.planar_teleop import PlanarMouseTeleop
from spartan.key_dynam.zmq_utils import ZMQClient


class KeyDynamController(PlanarMouseTeleop):

    def __init__(self, use_zmq=True):
        super(KeyDynamController, self).__init__()
        self._mouse_manager.release_mouse_focus()
        self._data_capture = DataCapture()
        self._data_capture.wait_for_initial_messages()

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
            print("\n\n---START BAGGING REQUEST---")
            self.start_bagging()
            print("bag started")
        elif pygame.locals.K_g in events["keydown"]:
            print("keydown:f")
            print("\n\n---STOP BAGGING REQUEST---")
            self.stop_bagging()
            print("bag stopped")
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
        elif pygame.locasl.K_e in events['keydown']:
            ### SEND SINGLE FRAME PLAN
            self.send_single_frame_plan()
            pass


    def send_single_frame_plan(self):
        """
        Capture data at single frame, send it over as a plan
        :return:
        """

        print("\n---SENDING PLAN----")

        plan_data = []

        data = self._data_capture.get_latest_data()

        # empty action msg
        ee_setpoint_msg = self.make_empty_cartesian_goal_point_msg()
        ee_setpoint_data = message_converter.convert_ros_message_to_dictionary(ee_setpoint_msg)
        data['actions'] = {'ee_setpoint': ee_setpoint_data}
        data['action'] = data['actions'] # for compatibility w/ key_dynam

        plan_data.append(data)

        msg = {'type': 'PLAN',
               'data': plan_data,
               }

        self._zmq_client.send_data(msg)
        self._zmq_client.recv_data()

    def send_control_action_req(self):
        data = self._data_capture.get_latest_data()


        print("\n---COMPUTE_CONTROL_ACTION REQUEST----")
        ee_setpoint_msg = self._task_space_streaming.get_last_published_message()
        if ee_setpoint_msg is None:
            ee_setpoint_msg = self.make_empty_cartesian_goal_point_msg()


        ee_setpoint_data = message_converter.convert_ros_message_to_dictionary(ee_setpoint_msg)
        data['actions'] = {'ee_setpoint': ee_setpoint_data}
        data['action'] = data['actions']  # for compatibility w/ key_dynam


        msg = {'type': 'COMPUTE_CONTROL_ACTION',
               'data': data,
               }

        self._zmq_client.send_data(msg)
        resp = self._zmq_client.recv_data()

    def run(self):
        """
        Runs main loop at given speed, 5 Hz
        :return:
        :rtype:
        """
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self.main_loop()
            rate.sleep()

    def debug_action(self):
        self.send_single_frame_plan()
        self.send_control_action_req()



if __name__ == "__main__":
    rospy.init_node('key_dynam_controller', anonymous=True)
    controller = KeyDynamController()
    controller.debug_action()
    print("Finished cleanly")
    # controller.run()