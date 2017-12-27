# system
import yaml
import time
import random
import os
import math
import numpy as np


# ROS
import rospy
import geometry_msgs.msg
import sensor_msgs.msg

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

    def waitForNextMessage(self):
        self.hasNewMessage = False
        while not self.hasNewMessage:
            rospy.sleep(0.1)
        return self.lastMsg

'''
Simple wrapper around the robot_control/MoveToJointPosition service
'''
class RobotService(object):

    def __init__(self, jointNames):
        self.jointNames = jointNames
        self.numJoints = len(jointNames)

    def moveToJointPosition(self, q, maxJointDegreesPerSecond=30):
        assert len(q) == self.numJoints

        jointState = RobotService.jointPositionToJointStateMsg(self.jointNames, q)

        rospy.wait_for_service('robot_control/MoveToJointPosition')
        s = rospy.ServiceProxy('robot_control/MoveToJointPosition', robot_msgs.srv.MoveToJointPosition)
        response = s(jointState, maxJointDegreesPerSecond)
        
        return response

    def moveToCartesianPosition(self, poseStamped, maxJointDegreesPerSecond=30):
        ikServiceName = 'robot_control/IkService'
        rospy.wait_for_service(ikServiceName)
        s = rospy.ServiceProxy(ikServiceName, robot_msgs.srv.RunIK)
        response = s(poseStamped)

        joint_state = response.joint_state

        rospy.loginfo("ik was successful = %s", response.success)

        if not response.success:
            rospy.loginfo("ik was not successful, returning without moving robot")
            return response.success

        rospy.loginfo("ik was successful, moving to joint position")
        return self.moveToJointPosition(joint_state.position, maxJointDegreesPerSecond=maxJointDegreesPerSecond)

    def runIK(self, poseStamped, seedPose=None, nominalPose=None):

        req = robot_msgs.srv.RunIKRequest()
        req.pose_stamped = poseStamped

        if seedPose:
            req.seed_pose.append(RobotService.jointPositionToJointStateMsg(self.jointNames, seedPose))

        if nominalPose:
            req.nominal_pose.append(RobotService.jointPositionToJointStateMsg(self.jointNames, nominalPose))

        ikServiceName = 'robot_control/IkService'
        rospy.wait_for_service(ikServiceName)
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
        jointNames = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',
     'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6',
     'iiwa_joint_7']

        return RobotService(jointNames)




