from __future__ import print_function

# system
import threading
import numpy as np
import os
import time
from PIL import Image
import copy

# ROS
import rospy
import sensor_msgs.msg
from rospy_message_converter import message_converter
import std_srvs.srv
import geometry_msgs.msg
import tf2_ros
from cv_bridge import CvBridge
import visualization_msgs.msg

# spartan
import robot_msgs.msg
from spartan.utils.ros_utils import SimpleSubscriber, RobotService
import spartan.utils.utils as spartan_utils
from spartan.manipulation.schunk_driver import SchunkDriver
import wsg_50_common.msg
from spartan.utils import transformations
from spartan.utils import constants

# imitation_tools
from imitation_tools.kinematics_utils import IiwaKinematicsHelper


# imitation_agent

DEBUG = True
PUBLISH = True

ee_tf_above_table = np.asarray([[-0.01389096,  0.38503428,  0.92279773,  0.51080192],
                                [ 0.04755021,  0.92209702, -0.38402613,  0.01549765],
                                [-0.99877226,  0.03854474, -0.03111728,  0.50188255],
                                [ 0.,          0.,          0.,          1.,       ]])


class ROSTaskSpaceControlAgent(object):
    """
    This class collects sensor data at 30Hz and stores it in a dict format
    similar to what is used as the state_dict in ImitationEpisode.

    note: (manuelli) will need to be a bit careful about how images are stored.
    No images for now, just proprioceptive information.

    To switch between different actual agents/network architectures the user simply
    needs to set the property `compute_control_input`. This function returns the control
    input as a ROS msg, namely as a robot_msgs.msg.CartesianGoalPoint
    """

    def __init__(self, tf_buffer=None, config=None):
        self._tf_buffer = tf_buffer
        self._config = config
        self._lock = threading.RLock() # re-entrant lock
        self._data_list = [] # list holding recent information for extracting input to policy
        self._max_data_list_size = 50 # max number of elements to store

        self._last_robot_joint_state_msg = None
        self._last_gripper_joint_state_msg = None

        # variable that holds imitation episode object
        # used for making the data that
        self._imitation_episode = None
        self.bridge = None

        self._robot_service = RobotService.makeKukaRobotService()
        self._gripper_driver = SchunkDriver()

        stored_poses_file  = os.path.join(spartan_utils.getSpartanSourceDir(), "src/catkin_projects/station_config/RLG_iiwa_1/stored_poses.yaml")
        self._stored_poses_dict = spartan_utils.getDictFromYamlFilename(stored_poses_file)

        self._tf2_broadcaster = tf2_ros.TransformBroadcaster()
        self._setup_robot_model()
        self.setup_subscribers()
        self.setup_publishers()
        self.control_rate = 30

    def convert_ros_to_numpy(self, rgb_image_ros):
        if self.bridge is None:
            self.bridge = CvBridge()

        cv_image = self.bridge.imgmsg_to_cv2(rgb_image_ros, "bgr8")
        numpy_img = cv_image[:, :, ::-1].copy() # open and convert between BGR and
        return numpy_img

    def _setup_robot_model(self):
        """
        Load an IiwaKinematicsHelper object
        :return:
        :rtype:
        """
        self._iiwa_kinematics_helper = IiwaKinematicsHelper()

    def setup_subscribers(self):
        """
        Sets up the ROS subscribers
        :return:
        :rtype:
        """
        # subscribe to image channel (RGB)
        self._robot_joint_states_subscriber = SimpleSubscriber("/joint_states", sensor_msgs.msg.JointState, externalCallback=self.on_joint_states_msg)
        self._robot_joint_states_subscriber.start()

        self._gripper_status_subscriber = SimpleSubscriber("/wsg50_driver/wsg50/status", wsg_50_common.msg.Status)
        self._gripper_status_subscriber.start()

        # self.camera_serial_number = "d415_02"
        # self._rgb_image_subscriber = SimpleSubscriber("/camera_"+self.camera_serial_number+"/color/image_raw", sensor_msgs.msg.Image)
        
        camera_name = "/camera_sim_d415_right/"
        self._rgb_image_subscriber = SimpleSubscriber(camera_name + "/rgb/image_rect_color", sensor_msgs.msg.Image)

        self._rgb_image_subscriber.start()

        self._object_pose_cheat_subscriber = SimpleSubscriber("/scene_graph/update", visualization_msgs.msg.InteractiveMarkerUpdate)
        self._object_pose_cheat_subscriber.start()


    def get_latest_images(self): # type -> dict
        msg = self._rgb_image_subscriber.last_message
        rgb_img_numpy = self.convert_ros_to_numpy(msg)

        data = dict()
        data['rgb_image'] = Image.fromarray(rgb_img_numpy).convert('RGB')

        # get depth image as well
        return data


    def setup_publishers(self):
        """
        Sets up the ROS publishers
        :return:
        :rtype:
        """
        self._control_publisher = rospy.Publisher('plan_runner/task_space_streaming_setpoint',
        robot_msgs.msg.CartesianGoalPoint, queue_size=1)


    def on_joint_states_msg(self, msg):
        """
        Deals with accepting both gripper and robot state joint messages
        :param msg:
        :type msg:
        :return:
        :rtype:
        """
        # if DEBUG:
        #     print("got /joint_states msg")

        if "iiwa_joint_1" in msg.name:
            self._last_robot_joint_state_msg = msg


    def compute_euler_from_homogenous_transform(self, T_W_E):
        quat = transformations.quaternion_from_matrix(T_W_E[0:3,0:3])
        quat = np.asarray(quat)
        
        def quat_to_euler(quat):
            x = transformations.quaternion_matrix(quat)
            T_delta = np.dot(x,np.linalg.inv(ee_tf_above_table))
            euler = transformations.euler_from_matrix(T_delta[0:3,0:3], 'syxz')
            return euler
        euler = quat_to_euler(quat)
        d = dict()
        d["pitch"] = euler[0]
        d["roll"] = euler[1]
        d["yaw"] = euler[2]
        return d


    def get_latest_data(self):
        """
        Grab the latest data, do forward kinematics if needed, insert it into the data list
        :return:
        :rtype: bool, dict
        """

        data = dict()
        if self._last_robot_joint_state_msg is None:
            if DEBUG:
                print("no robot state msg found")
            return False, data

        if self._gripper_status_subscriber.last_message is None:
            if DEBUG:
                print("no gripper message found")
            return False, data

        data['observations'] = dict()
        obs = data['observations']

        q = np.array(self._last_robot_joint_state_msg.position)
        T_W_E = self._iiwa_kinematics_helper.compute_ee_forward_kinematics(q)
        obs['ee_to_world'] = spartan_utils.dict_from_homogenous_transform(T_W_E)
        obs['gripper_state'] = message_converter.convert_ros_message_to_dictionary(self._gripper_status_subscriber.last_message)

        obs['ee_to_world']["rpy"] = self.compute_euler_from_homogenous_transform(T_W_E)


        # object_pose = self.build_object_pose_dict()
        # obs['object_pose_cheat_data'] = object_pose

        data["T_W_C"] = np.matmul(T_W_E, constants.T_E_cmd)


        return True, data

    @property
    def control_function(self):
        return self._control_function

    @control_function.setter
    def control_function(self, val):
        self._control_function = val

    @property
    def imitation_episode(self):
        return self._imitation_episode


    @property
    def data_list(self):
        return self._data_list

    def compute_control_action(self):
        """
        Call control function, publish resulting ROS message (after some safety checks . . . )
        :return:
        :rtype:
        """
        msg = self.control_function(self)
        # apply safety checks???
        return msg

    def move_to_starting_position(self):
        """
        Move the robot home, set the gripper
        :return:
        :rtype:
        """
        above_table_pre_grasp = self._stored_poses_dict["Grasping"]["above_table_pre_grasp"]

        #slightly_lower = [0.049923915416002274, 0.3583604395389557,-0.03587953373789787, -1.6347194910049438, 0.06593543291091919, 1.1629064083099365, 0.3850176930427551]

        success = self._robot_service.moveToJointPosition(above_table_pre_grasp, maxJointDegreesPerSecond=30, timeout=5)
        
        gripper_goal_pos = 0.0
        time.sleep(1.0)
        self._gripper_driver.sendGripperCommand(gripper_goal_pos, speed=0.2, timeout=0.01)
        print("sent close goal to gripper")
        time.sleep(2.0)
        gripper_goal_pos = 0.1
        self._gripper_driver.sendGripperCommand(gripper_goal_pos, speed=0.2, timeout=0.01)
        print("sent open goal to gripper")
        time.sleep(2.0)



    def run(self):

        # start the TaskSpaceStreamingController
        service_proxy = rospy.ServiceProxy('plan_runner/init_task_space_streaming',
        robot_msgs.srv.StartStreamingPlan)

        init = robot_msgs.srv.StartStreamingPlanRequest()
        res = service_proxy(init)

        rate = rospy.Rate(self.control_rate)  # 30 Hz rate

        def cleanup():
            rospy.wait_for_service("plan_runner/stop_plan")
            sp = rospy.ServiceProxy('plan_runner/stop_plan',
                                    std_srvs.srv.Trigger)
            init = std_srvs.srv.TriggerRequest()
            sp(init)
            print("Done cleaning up and stopping streaming plan")

        rospy.on_shutdown(cleanup)

        start_time = time.time()
        while not rospy.is_shutdown():
            # grab the lock
            try:
                self._lock.acquire()
                self.step() 

            finally:
                self._lock.release()

            rate.sleep()


    def step(self):
        valid, data = self.get_latest_data()

        if not valid:
            if DEBUG:
                print("data wasn't valid, skipping")
            return

        self._data_list.append(data)
        self._data_list = self._data_list[-self._max_data_list_size:]

        msg, gripper_goal = self.compute_control_action()
        if PUBLISH:
            self._control_publisher.publish(msg)
            self._gripper_driver.sendGripperCommand(gripper_goal, speed=0.2, stream=True)
           
        self.visualize_command_frame(msg)
        # publish control input




    def visualize_command_frame(self, msg):
        """
        Publishes command frame to TF if in position control mode
        :return:
        :rtype:
        """
        #if msg.use_end_effector_velocity_mode:
        #    return

        tf_stamped = geometry_msgs.msg.TransformStamped()
        tf_stamped.header.stamp = rospy.Time.now()
        tf_stamped.header.frame_id = "base"
        tf_stamped.child_frame_id = "task_space_control_goal"
        tf_stamped.transform.translation.x = msg.xyz_point.point.x
        tf_stamped.transform.translation.y = msg.xyz_point.point.y
        tf_stamped.transform.translation.z = msg.xyz_point.point.z

        tf_stamped.transform.rotation.x = msg.quaternion.x
        tf_stamped.transform.rotation.y = msg.quaternion.y
        tf_stamped.transform.rotation.z = msg.quaternion.z
        tf_stamped.transform.rotation.w = msg.quaternion.w

        self._tf2_broadcaster.sendTransform(tf_stamped)


    @staticmethod
    def default_config():
        config = dict()
        config["control_publish_channel"] = "/plan_runner/"
        return config