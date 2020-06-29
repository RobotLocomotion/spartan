from __future__ import print_function

import os
from enum import Enum
import time
import pygame


# ROS
import rospy
import std_srvs.srv

# imitation_tools

# spartan
from spartan.utils.control_utils import make_cartesian_gains_msg
from spartan.key_dynam.teleop_mouse_manager import TeleopMouseManager
from spartan.key_dynam.utils import start_bagging_imitation_data_client, stop_bagging_imitation_data_client
import spartan.utils.utils as spartan_utils
from spartan.utils.ros_utils import JointStateSubscriber, RobotService
import robot_msgs.msg


def test_teleop_mouse_manager():

    hz = 1
    mouse_manager = TeleopMouseManager()
    start_time = time.time()
    mouse_to_velocity_scale = 0.0001
    try:
        while (time.time() - start_time) < 15:
            events = mouse_manager.get_events()
            print("mouse dx, dy", (events['delta_x'], events['delta_y']))
            vx = mouse_to_velocity_scale * events['delta_x']
            vy = mouse_to_velocity_scale * events['delta_y']
            print("vx, vy", (vx, vy))
            time.sleep(1/hz)
    except:
        print("Suffered exception")


class MouseTeleopState(Enum):
    STOPPED = 1
    RUNNING = 2

class MouseTeleop(object):
    """
    List out control keys

    - s = start streaming
    - t = stop streaming
    - f = start bagging
    - g = stop bagging
    - esc = quit/shutdown
    - r = reset (stop streaming, stop bagging, move to "above_table_ready_pose", then "start_pose_left"
    - e, w (resets to different poses)

    SPECIAL KEYS: from TeleopMouseManager()
    - RETURN: grab mouse focus
    - SPACE: release mouse focus
    - ESC: release mouse focus

    """



    def __init__(self, rate=5, kp_rot=5, kp_trans=10):
        self._rate = rate
        self._mouse_manager = TeleopMouseManager() # note this grabs mouse focus
        # self._mouse_manager.release_mouse_focus()

        self.setup_publishers()
        self.setup_subscribers()
        self.setup_service_proxies()
        self.wait_for_robot_state()

        # movement service
        self._robotService = RobotService.makeKukaRobotService()

        self._state = MouseTeleopState.STOPPED
        self._kp_rot = kp_rot
        self._kp_trans = kp_trans
        rospy.on_shutdown(self.on_shutdown)

        self._stored_poses_dict = spartan_utils.getDictFromYamlFilename(os.path.join(spartan_utils.getSpartanSourceDir(), "src/catkin_projects/station_config/RLG_iiwa_1/stored_poses.yaml"))

    def reset(self, key=None):
        raise NotImplementedError("subclass must implement this method")

    def setup_publishers(self):
        """
        Setup the publishers
        :return:
        :rtype:
        """
        self._task_space_streaming_pub = rospy.Publisher('plan_runner/task_space_streaming_setpoint',
                              robot_msgs.msg.CartesianGoalPoint, queue_size=1)


    def setup_subscribers(self):
        """
        Setup the subscribers
        :return:
        :rtype:
        """
        self.robotSubscriber = JointStateSubscriber("/joint_states")


    def setup_service_proxies(self):
        """
        Setup the service proxies
        :return:
        :rtype:
        """
        rospy.wait_for_service('plan_runner/init_task_space_streaming')
        self._init_task_space_streaming_proxy = rospy.ServiceProxy('plan_runner/init_task_space_streaming',
        robot_msgs.srv.StartStreamingPlan)

        rospy.wait_for_service("plan_runner/stop_plan")
        self._stop_plan_service_proxy = sp = rospy.ServiceProxy('plan_runner/stop_plan',
            std_srvs.srv.Trigger)

    def wait_for_robot_state(self):
        """
        Waits until it receives a message on the /joint_states channel with
        the appropriate number of entries
        :return:
        :rtype:
        """
        print("Waiting for full kuka state...")
        while len(self.robotSubscriber.joint_positions.keys()) < 3:
            rospy.sleep(0.1)
        print("got full state")

    def start_teleop(self):
        """
        Start the streaming plan
        :return:
        :rtype:
        """
        if not self._state == MouseTeleopState.STOPPED:
            # can only call this from the stopped state
            print("called start_teleop while state was %s", self._state)
            print('ignoring and returning')
            return


        print("\n\n-----STARTING TELEOP----")
        self._mouse_manager.grab_mouse_focus()

        init_msg = robot_msgs.srv.StartStreamingPlanRequest()
        res = self._init_task_space_streaming_proxy(init_msg)

        self._state = MouseTeleopState.RUNNING

    def stop_teleop(self):
        """
        Stop the streaming plan
        :return:
        :rtype:
        """
        print("stopping teleop plan")
        self._stop_plan_service_proxy(std_srvs.srv.TriggerRequest())
        self._state = MouseTeleopState.STOPPED

    def start_bagging(self):
        """
        Start ros bag
        :return:
        :rtype:
        """
        start_bagging_imitation_data_client()


    def stop_bagging(self):
        """
        Stop ros bag
        :return:
        :rtype:
        """

        stop_bagging_imitation_data_client()

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


        # if it's running, then send out the message
        if self._state == MouseTeleopState.RUNNING:
            msg = self.get_task_space_streaming_message(events)
            self._task_space_streaming_pub.publish(msg)

        # print("heartbeat")


    def get_task_space_streaming_message(self, events):
        raise NotImplementedError("subclass must implement this")

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


    def on_shutdown(self):
        """
        Actions to take on shutdown

        - stop bagging
        - stop streaming plan, etc.
        :return:
        :rtype:
        """

        # stop any plan if it is running
        self.stop_teleop()
        self.stop_bagging()
        # self._stop_plan_service_proxy(std_srvs.srv.TriggerRequest())


    def make_empty_cartesian_goal_point_msg(self):
        """
        Makes a cartesian_goal_point_msg with zero velocity
        :return:
        :rtype:
        """
        msg = robot_msgs.msg.CartesianGoalPoint()

        msg.ee_frame_id = "iiwa_link_ee"
        msg.gain = make_cartesian_gains_msg(kp_rot=self._kp_rot, kp_trans=self._kp_trans)

        msg.setpoint_linear_velocity_mode = True
        msg.setpoint_linear_velocity.x = 0.
        msg.setpoint_linear_velocity.y = 0.
        msg.setpoint_linear_velocity.z = 0.

        return msg


class PlanarMouseTeleop(MouseTeleop):

    def __init__(self):
        super(PlanarMouseTeleop, self).__init__()
        self._velocity_scale_factor = 1e-3


    def get_task_space_streaming_message(self, events):

        v_x = events['delta_y'] * self._velocity_scale_factor
        v_y = events['delta_x'] * self._velocity_scale_factor

        print("v_x, v_y", (v_x, v_y))
        msg = self.make_empty_cartesian_goal_point_msg()
        msg.setpoint_linear_velocity.x = v_x
        msg.setpoint_linear_velocity.y = v_y

        return msg

    def reset(self, key='r'):
        """
        Resets for the next data collection

        - stop teleop
        - stop bagging
        - move to safe position then move to start position

        :return:
        """
        self.stop_teleop()
        self.stop_bagging()

        # move to safe pose above table
        pose = self._stored_poses_dict["key_dynam"]["above_table_ready_pose"]
        success = self._robotService.moveToJointPosition(pose, maxJointDegreesPerSecond=30, timeout=5)

        # move to start pose
        pose = None
        if key == 'r':
            pose = self._stored_poses_dict["key_dynam"]["start_pose_left"]
        elif key == 'e':
            pose = self._stored_poses_dict["key_dynam"]["start_pose_right"]
        elif key == 'w':
            pose = self._stored_poses_dict["key_dynam"]["start_pose_center_back"]
        else:
            raise ValueError("unknown key type in reset")

        success = self._robotService.moveToJointPosition(pose, maxJointDegreesPerSecond=30, timeout=5)





if __name__ == "__main__":
    rospy.init_node('planar_teleop', anonymous=True)
    teleop = PlanarMouseTeleop()
    teleop.run()
    # test_teleop_mouse_manager()