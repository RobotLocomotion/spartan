# system
import yaml
import time
import random
import os
import math


# ROS
import rospy
import geometry_msgs.msg

# spartan
import spartan.utils.utils as spartanUtils



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