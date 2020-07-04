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
from spartan.utils import ros_utils
import spartan.utils.utils as spartan_utils
from spartan.manipulation.schunk_driver import SchunkDriver
import wsg_50_common.msg
from spartan.utils import transformations
from spartan.utils import constants
from spartan.key_dynam.zmq_utils import ZMQClient

# imitation_tools
from imitation_tools.kinematics_utils import IiwaKinematicsHelper



class ROSTaskSpaceControlAgent(object):
    """
    Collects and sends data over ZMQ to the key_dynam node on the Python3 side
    """

    def __init__(self,
                 tf_buffer=None,
                 config=None,
                 ):
        self._tf_buffer = tf_buffer
        self._config = config

        self._zmq_client = ZMQClient()

        self.bridge = CvBridge()

        self._robot_service = RobotService.makeKukaRobotService()

        stored_poses_file  = os.path.join(spartan_utils.getSpartanSourceDir(), "src/catkin_projects/station_config/RLG_iiwa_1/stored_poses.yaml")
        self._stored_poses_dict = spartan_utils.getDictFromYamlFilename(stored_poses_file)

        self._setup_robot_model()
        self.setup_subscribers()

        rospy.on_shutdown(self.on_shutdown)

        self.wait_for_initial_messages()

    def on_shutdown(self):
        pass

    def wait_for_initial_messages(self):

        for key, subscriber in self._subscriber_dict.iteritems():
            subscriber.waitForNextMessage()


    def convert_ros_to_numpy(self, rgb_image_ros):
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
        self._subscriber_dict = dict()

        # # subscribe to image channel (RGB)
        # self._robot_joint_states_subscriber = SimpleSubscriber("/joint_states", sensor_msgs.msg.JointState, externalCallback=self.on_joint_states_msg)
        # self._robot_joint_states_subscriber.start()

        # subscribe to color image
        self.camera_serial_number = "d415_01"
        self._rgb_image_subscriber = SimpleSubscriber("/camera_"+self.camera_serial_number+"/color/image_raw", sensor_msgs.msg.Image)
        self._rgb_image_subscriber.start()

        self._subscriber_dict['rgb'] = self._rgb_image_subscriber

        # subscribe to depth image
        self._depth_image_subscriber = SimpleSubscriber("/camera_"+self.camera_serial_number+"/aligned_depth_to_color/image_raw", sensor_msgs.msg.Image)
        self._depth_image_subscriber.start()
        self._subscriber_dict['depth'] = self._depth_image_subscriber

    def get_latest_image_data(self):
        rgb_msg = self._rgb_image_subscriber.last_message

        rgb = ros_utils.rgb_image_to_cv2_uint8(rgb_msg, bridge=self.bridge)
        # print("rgb.dtype", rgb.dtype)

        depth_msg = self._depth_image_subscriber.last_message
        depth = ros_utils.depth_image_to_cv2_uint16(depth_msg, bridge=self.bridge,
                                                    encoding="passthrough")

        # print("depth.dtype", depth.dtype)

        image_data = {'rgb': rgb,
                      'depth_int16': depth,
                      }
        image_data['rgb'] = rgb

        return image_data

    def get_latest_data(self):
        """


        returns a dict with keys
        ['observations', 'actions', 'control']
        :return:
        """

        # return a dict with the latest observation data

        data = dict()
        data['observations'] = dict()
        data['observations']['images'] = dict()
        data['observations']['images'][self.camera_serial_number] = self.get_latest_image_data()

        return data

    def get_latest_images(self): # type -> dict
        msg = self._rgb_image_subscriber.last_message
        rgb_img_numpy = self.convert_ros_to_numpy(msg)

        data = dict()
        data['rgb_image'] = Image.fromarray(rgb_img_numpy).convert('RGB')

        # get depth image as well
        return data

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

    def compute_control_action(self):
        """
        Call control function, publish resulting ROS message (after some safety checks . . . )
        :return:
        :rtype:
        """
        raise NotImplementedError
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

        rate = rospy.Rate(self._config['rate'])

        while not rospy.is_shutdown():

            data = self.get_latest_data()
            start_time = time.time()
            self._zmq_client.send_data(data)
            resp = self._zmq_client.recv_data()
            elapsed = time.time() - start_time
            print("elapsed time:", elapsed)
            rate.sleep()

    @staticmethod
    def default_config():
        config = dict()
        config["control_publish_channel"] = "/plan_runner/"
        return config


if __name__ == "__main__":
    config_file = os.path.join(spartan_utils.getSpartanSourceDir(), 'modules/spartan/key_dynam/controller_config.yaml')
    config = spartan_utils.getDictFromYamlFilename(config_file)
    rospy.init_node("key_dynam_controller")
    controller = ROSTaskSpaceControlAgent(config=config)
    controller.run()