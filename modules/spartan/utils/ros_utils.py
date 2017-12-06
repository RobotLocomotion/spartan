# system
import yaml
import time
import random
import os
import math


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


class SimpleSubscriber(object):
    def __init__(self, topic, messageType, externalCallback=None):
        self.topic = topic
        self.messageType = messageType
        self.externalCallback = externalCallback
        self.hasNewMessage = False
        self.lastMsg = None

    def start(self):
        self.subscriber = rospy.Subscriber(self.topic, self.messageType, self.callback)

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

        jointState = sensor_msgs.msg.JointState()
        jointState.header.stamp = rospy.Time.now()

        jointState.position = q
        jointState.name = self.jointNames

        jointState.velocity = [0] * self.numJoints
        jointState.effort = [0] * self.numJoints

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

    def runIK(self, poseStamped):
        ikServiceName = 'robot_control/IkService'
        rospy.wait_for_service(ikServiceName)
        s = rospy.ServiceProxy(ikServiceName, robot_msgs.srv.RunIK)
        response = s(poseStamped)

        joint_state = response.joint_state

        rospy.loginfo("ik was successful = %s", response.success)
        return response



