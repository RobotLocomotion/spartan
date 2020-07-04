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



class DataCapture(object):
    """
    Collects data needed for doing MPC controller
    """

    def __init__(self,
                 tf_buffer=None,
                 config=None,
                 ):

        self._config = config

        if tf_buffer is None:
            tf_buffer = tf2_ros.Buffer()

        self._tf_buffer = tf_buffer
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._subscribers_list = []

        self.bridge = CvBridge()

        self.camera_names = ["d415_01"]
        self.setup_subscribers()
        self.world_frame = "base"
        self.end_effector_frame = "iiwa_link_ee"

        rospy.on_shutdown(self.on_shutdown)
        self.wait_for_initial_messages()

    def on_shutdown(self):
        pass

    def wait_for_initial_messages(self):
        # for key, subscriber in self._subscriber_dict.iteritems():
        #     subscriber.waitForNextMessage()

        print("waiting for initial messages")
        for subscriber in self._subscribers_list:
            subscriber.waitForNextMessage()

        print("initial messages received")
    def setup_subscribers(self):

        """
        Sets up the ROS subscribers
        :return:
        :rtype:
        """
        self._image_subscribers = dict()
        for camera_name in self.camera_names:
            rgb_sub = SimpleSubscriber("/camera_"+camera_name+"/color/image_raw", sensor_msgs.msg.Image)
            rgb_sub.start()

            depth_sub = SimpleSubscriber("/camera_"+camera_name+"/aligned_depth_to_color/image_raw", sensor_msgs.msg.Image)
            depth_sub.start()

            self._image_subscribers[camera_name] = {'rgb': rgb_sub,
                                                    'depth': depth_sub,}
            self._subscribers_list.append(rgb_sub)
            self._subscribers_list.append(depth_sub)

    def get_latest_image_data(self):
        image_data = dict()

        for camera_name in self.camera_names:
            rgb_msg = self._image_subscribers[camera_name]['rgb'].last_message
            rgb = ros_utils.rgb_image_to_cv2_uint8(rgb_msg, bridge=self.bridge)

            depth_msg = self._image_subscribers[camera_name]['depth'].last_message
            depth = ros_utils.depth_image_to_cv2_uint16(depth_msg, bridge=self.bridge,
                                                        encoding="passthrough")

            image_data[camera_name] = {'rgb': rgb,
                                       'depth_16U': depth,
                                       'depth_int16': depth,
                                       }

        return image_data

    def get_ee_to_world(self):
        """
        Get current position of end-effector
        :return:
        """

        pos, quat = ros_utils.poseFromROSTransformMsg(
            self._tf_buffer.lookup_transform(self.world_frame, self.end_effector_frame, rospy.Time()).transform)

        transform_dict = spartan_utils.dictFromPosQuat(pos, quat)
        return transform_dict

    def get_latest_data(self):
        """


        returns a dict with keys 'observations'. The goal is to mimic
        the format in the states.yaml that is extracted from the logs

        - observations
            - images
                - d415_01
                    - rgb (the rgb image)
                    - depth_16U (depth image)
                    - depth_int16 (depth image)
            - ee_to_world:
              quaternion:
                w: 0.6864535407987857
                x: 0.1471408294834778
                y: 0.6998186048673694
                z: -0.13185231475105652
              translation:
                x: 0.5975213794365257
                y: -0.25454892573241006
                z: 0.18706437015937702
        :return:


        """

        # return a dict with the latest observation data

        data = dict()
        data['observations'] = dict()
        data['observations']['images'] = self.get_latest_image_data()

        data['observations']['ee_to_world'] = self.get_ee_to_world()
        data['observations']['timestamp'] = rospy.Time.now().to_nsec()
        data['observations']['timestamp_system'] = time.time()

        data['observation'] = data['observations'] # for compatibility w/ key_dynam

        return data
