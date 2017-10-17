import yaml
import time
import random
import os
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform

from director import transformUtils


# ROS
import rospy

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