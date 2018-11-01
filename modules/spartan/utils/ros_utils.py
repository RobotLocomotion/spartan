# system
import yaml
import time
import random
import os
import math
import numpy as np

import cv2


# ROS
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge
import tf


# spartan
import spartan.utils.utils as spartanUtils
import robot_msgs.srv


def ROSPoseMsgFromPose(d):
    msg = geometry_msgs.msg.Pose()
    msg.position.x = d['translation']['x']
    msg.position.y = d['translation']['y']
    msg.position.z = d['translation']['z']

    quatDict = spartanUtils.getQuaternionFromDict(d)

    msg.orientation.w = quatDict['w']
    msg.orientation.x = quatDict['x']
    msg.orientation.y = quatDict['y']
    msg.orientation.z = quatDict['z']

    return msg

def ROSTransformMsgFromPose(d):
    msg = geometry_msgs.msg.Transform()
    msg.translation.x = d['translation']['x']
    msg.translation.y = d['translation']['y']
    msg.translation.z = d['translation']['z']

    quatDict = spartanUtils.getQuaternionFromDict(d)

    msg.rotation.w = quatDict['w']
    msg.rotation.x = quatDict['x']
    msg.rotation.y = quatDict['y']
    msg.rotation.z = quatDict['z']

    return msg

def dictToPointMsg(d):
    msg = geometry_msgs.msg.Point()
    msg.x = d['x']
    msg.y = d['y']
    msg.z = d['z']

    return msg

def listToPointMsg(l):
    msg = geometry_msgs.msg.Point()
    
    msg.x = l[0]
    msg.y = l[1]
    msg.z = l[2]

    return msg

def arrayToPointMsgs(pts):
    # Takes 3xN array of points,
    # and produces list of Point messages
    
    return [geometry_msgs.msg.Point(
        pts[0, i], pts[1, i], pts[2, i])
        for i in range(pts.shape[1])]

"""
@param msg: geometry_msgs.msg.Point
@return list of [x,y,z] position
"""
def pointMsgToList(msg):
    l = []
    l.append(msg.x)
    l.append(msg.y)
    l.append(msg.z)
    return l

"""
@param msg: geometry_msgs.msg.Quaternion
@return list: [w,x,y,z]
"""
def quatMsgToList(msg):
    quat = []
    quat.append(msg.w)
    quat.append(msg.x)
    quat.append(msg.y)
    quat.append(msg.z)

    return quat


def poseFromROSTransformMsg(msg):
    pos = [msg.translation.x, msg.translation.y, msg.translation.z]
    quat = [msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z]
    return pos, quat

def camera_info_dict_from_camera_info_msg(msg):
    d = dict()

    d['camera_matrix'] = dict()
    d['camera_matrix']['cols'] = 3
    d['camera_matrix']['rows'] = 3
    d['camera_matrix']['data'] = list(msg.K)

    d['distortion_model'] = msg.distortion_model

    d['distortion_coefficients'] = dict()
    d['distortion_coefficients']['cols'] = 5
    d['distortion_coefficients']['rows'] = 1
    d['distortion_coefficients']['data'] = list(msg.D)

    d['projection_matrix'] = dict()
    d['projection_matrix']['cols'] = 4
    d['projection_matrix']['rows'] = 4
    d['projection_matrix']['data'] = list(msg.P)


    d['rectification_matrix'] = dict()
    d['rectification_matrix']['cols'] = 3
    d['rectification_matrix']['rows'] = 3
    d['rectification_matrix']['data'] = list(msg.R)

    d['image_width'] = msg.width
    d['image_height'] = msg.height

    return d


"""
Convert pointcloud from 32FC to 16UC format
See the discussions
http://www.ros.org/reps/rep-0117.html and 
http://www.ros.org/reps/rep-0118.html 
for how pointclouds are encoded

NaN: invalid measurement, could be coming from padding from registering depth to rgb
-Inf: too close to measure, not missing
Inf: max range

any ranges above maxRange --> 10**16
Nan --> 10**16 - 1
"""
def convert32FCto16UC(img_in, maxRange=5):
    
    # first set the nan's to zero
    img = np.copy(img_in)
    info = np.iinfo(np.uint16)

    # pos_inf_idx = np.isposinf(img)
    above_max_range_idx = img > maxRange
    neg_inf_idx = np.isneginf(img)
    nan_idx = np.isnan(img)

    # scale to the maxRange
    np.clip(img, 0, maxRange)

    # convert to decimillimeters 10^-4 of a meter
    img_scaled = img*10**4
    img_int = img_scaled.astype(np.uint16)
    img_int[above_max_range_idx] = iinfo.max
    img_int[nan_idx] = iinfo.max - 1
    return img_int

def depth_image_to_cv2_uint16(depth_image_msg, bridge=None):
    if bridge is None:
        bridge = CvBridge()
    """
    Parameters:
        depth_image_msg: sensor_msgs.Image
    """

    cv_img = bridge.imgmsg_to_cv2(depth_image_msg, "32FC1")
    cv_img = np.array(cv_img, dtype=np.float32)
    cv_img = cv_img*1000
    cv_img = cv_img.astype(np.uint16)

    return cv_img

"""
Saves a single image to a filename using an external executable
"""

def saveSingleImage(topic, filename, encoding=None):
        rosImageLoggerExecutable = os.path.join(spartanUtils.getSpartanSourceDir(), 'modules',"spartan",
                                                'calibration','ros_image_logger.py')
        cmd = "%s -t %s -f %s" % (rosImageLoggerExecutable, topic, filename)
        if encoding is not None:
            cmd += " -e " + encoding

        os.system(cmd)

"""
Saves a single image to a filename using an external executable
"""

def saveSingleDepthImage(topic, filename, encoding=None):
        rosImageLoggerExecutable = os.path.join(spartanUtils.getSpartanSourceDir(), 'modules',"spartan",
                                                'calibration','ros_image_logger.py')
        cmd = "%s -t %s -f %s" % (rosImageLoggerExecutable, topic, filename)
        if encoding is not None:
            cmd += " -e " + encoding

        cmd += " -fs"

        os.system(cmd)

def getRGBOpticalFrameName(camera_name):
    return "camera_" +  camera_name + "_rgb_optical_frame"


def setup_tf_transformer_from_ros_bag(bag, cache_time_secs=3600, verbose=False):
    """
    Creates a tf::Transformer object to allow for querying tf. Builds it
    with messages from a log
    """
    tf_t = tf.Transformer(True, rospy.Duration(secs=cache_time_secs))

    tf_static_msgs = []
    timestamps = []


    tf_set = set()


    for topic, msg, t in bag.read_messages(topics=['/tf_static']):
        for msg_tf in msg.transforms:
            child_frame_id = msg_tf.child_frame_id
            parent_frame_id = msg_tf.header.frame_id
            key = (child_frame_id, parent_frame_id)
            if key in tf_set:
                continue

            tf_set.add(key)
            tf_t.setTransform(msg_tf)

            
            tf_static_msgs.append(msg_tf)

        timestamps.append(t)


    
    # raise ValueError("donezos")
    def addStaticMessagesToTransformer(stamp):
        if verbose:
            print "len(tf_static_msgs): ", len(tf_static_msgs)

        for msg in tf_static_msgs:
            msg.header.stamp = stamp
            tf_t.setTransform(msg)

    
    counter = 0
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        # print "transform msg: \n", msg
        # print "type(msg)", type(msg)
        stamp = None
        counter += 1
        if verbose:
            print "processing tf message %d" %(counter)

        for msg_tf in msg.transforms:
            # if ("base" == msg_tf.child_frame_id) or ("base" == msg_tf.header.frame_id):
            #     print "timestamp = ", msg_tf.header.stamp.to_sec()
            #     print msg_tf

            # print "type(msg_tf)", type(msg_tf)
            tf_t.setTransform(msg_tf)
            stamp = msg_tf.header.stamp

        addStaticMessagesToTransformer(stamp)

    return tf_t


class SimpleSubscriber(object):
    def __init__(self, topic, messageType, externalCallback=None):
        self.topic = topic
        self.messageType = messageType
        self.externalCallback = externalCallback
        self.hasNewMessage = False
        self.lastMsg = None

    def start(self, queue_size=None):
        self.subscriber = rospy.Subscriber(self.topic, self.messageType, self.callback, queue_size=queue_size)
        
    def stop(self):
        self.subscriber.unregister()

    def callback(self, msg):
        self.lastMsg = msg
        self.hasNewMessage = True

        if self.externalCallback is not None:
            self.externalCallback(msg)

    def waitForNextMessage(self, sleep_duration=0.1):
        self.hasNewMessage = False
        while not self.hasNewMessage:
            rospy.sleep(sleep_duration)
        return self.lastMsg

    @property
    def last_message(self):
        return self.lastMsg


class JointStateSubscriber(object):
    ''' Subscribes to a joint state channel (by default, /joint_states),
        and keeps a dictionary of last-known joint values and the
        last time those values were updated. '''
    def __init__(self, topic="/joint_states"):
        self.topic = topic
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_efforts = {}
        self.joint_timestamps = {}
        self.subscriber = SimpleSubscriber(
            self.topic, sensor_msgs.msg.JointState, self.callback)
        self.subscriber.start()

    def callback(self, msg):
        stamp = msg.header.stamp
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
            self.joint_velocities[name] = msg.velocity[i]
            self.joint_efforts[name] = msg.effort[i]
            self.joint_timestamps[name] = stamp

    def get_position_vector_from_joint_names(self, joint_name_list):
        q = np.zeros(len(joint_name_list))
        for i, name in enumerate(joint_name_list):
            if name not in self.joint_positions.keys():
                raise ValueError("Never received state for joint %s" % name)
            q[i] = self.joint_positions[name]
        return q

    def get_velocity_vector_from_joint_names(self, joint_name_list):
        v = np.zeros(len(joint_name_list))
        for i, name in enumerate(joint_name_list):
            if name not in self.joint_positions.keys():
                raise ValueError("Never received state for joint %s" % name)
            v[i] = self.joint_velocities[name]
        return v

    def get_effort_vector_from_joint_names(self, joint_name_list):
        e = np.zeros(len(joint_name_list))
        for i, name in enumerate(joint_name_list):
            if name not in self.joint_positions.keys():
                raise ValueError("Never received state for joint %s" % name)
            e[i] = self.joint_efforts[name]
        return e


'''
Simple wrapper around the robot_control/MoveToJointPosition service
'''
class RobotService(object):

    def __init__(self, jointNames):
        self.jointNames = jointNames
        self.numJoints = len(jointNames)

    def moveToJointPosition(self, q, maxJointDegreesPerSecond=30, timeout=10):
        assert len(q) == self.numJoints

        jointState = RobotService.jointPositionToJointStateMsg(self.jointNames, q)

        rospy.wait_for_service('robot_control/MoveToJointPosition', timeout=timeout)
        s = rospy.ServiceProxy('robot_control/MoveToJointPosition', robot_msgs.srv.MoveToJointPosition)
        response = s(jointState, maxJointDegreesPerSecond)
        
        return response

    def moveToCartesianPosition(self, poseStamped, maxJointDegreesPerSecond=30, timeout=10):
        ikServiceName = 'robot_control/IkService'
        rospy.wait_for_service(ikServiceName, timeout=timeout)
        s = rospy.ServiceProxy(ikServiceName, robot_msgs.srv.RunIK)
        response = s(poseStamped)

        joint_state = response.joint_state

        rospy.loginfo("ik was successful = %s", response.success)

        if not response.success:
            rospy.loginfo("ik was not successful, returning without moving robot")
            return response.success

        rospy.loginfo("ik was successful, moving to joint position")
        return self.moveToJointPosition(joint_state.position, maxJointDegreesPerSecond=maxJointDegreesPerSecond)

    def runIK(self, poseStamped, seedPose=None, nominalPose=None, timeout=10):

        req = robot_msgs.srv.RunIKRequest()
        req.pose_stamped = poseStamped

        if seedPose:
            req.seed_pose.append(RobotService.jointPositionToJointStateMsg(self.jointNames, seedPose))

        if nominalPose:
            req.nominal_pose.append(RobotService.jointPositionToJointStateMsg(self.jointNames, nominalPose))

        ikServiceName = 'robot_control/IkService'
        rospy.wait_for_service(ikServiceName, timeout=timeout)
        s = rospy.ServiceProxy(ikServiceName, robot_msgs.srv.RunIK)
        response = s(req)

        joint_state = response.joint_state

        rospy.loginfo("ik was successful = %s", response.success)
        return response

    @staticmethod
    def jointPositionToJointStateMsg(jointNames, jointPositions):
        assert len(jointNames) == len(jointPositions)

        numJoints = len(jointNames)

        jointState = sensor_msgs.msg.JointState()
        jointState.header.stamp = rospy.Time.now()

        jointState.position = jointPositions
        jointState.name = jointNames

        jointState.velocity = [0] * numJoints
        jointState.effort = [0] * numJoints

        return jointState

    @staticmethod
    def makeKukaRobotService():
        return RobotService(spartanUtils.get_kuka_joint_names())
