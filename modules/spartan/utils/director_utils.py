__author__ = 'manuelli'
import numpy as np
import collections
import yaml
import os

from collections import namedtuple

from director.timercallback import TimerCallback
from director import robotstate
import drake as lcmdrake
from director import lcmUtils
from director import utime as utimeUtil
from director import transformUtils


def poseFromTransform(transform):
    pos, quat = transformUtils.poseFromTransform(transform)
    pos = pos.tolist()
    quat = quat.tolist()
    d = dict()
    d['translation'] = dict()
    d['translation']['x'] = pos[0]
    d['translation']['y'] = pos[1]
    d['translation']['z'] = pos[2]

    d['quaternion'] = dict()
    d['quaternion']['w'] = quat[0]
    d['quaternion']['x'] = quat[1]
    d['quaternion']['y'] = quat[2]
    d['quaternion']['z'] = quat[3]

    return d

def transformFromPose(d):
    pos = [0]*3
    pos[0] = d['translation']['x']
    pos[1] = d['translation']['y']
    pos[2] = d['translation']['z']

    quat = [0] * 4
    quat[0] = d['quaternion']['w']
    quat[1] = d['quaternion']['x']
    quat[2] = d['quaternion']['y']
    quat[3] = d['quaternion']['z']

    return transformUtils.transformFromPose(pos, quat)


class EstRobotStatePublisher(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.timer = TimerCallback(targetFps=25)
        self.timer.callback = self.publishEstRobotState

    def publishEstRobotState(self):
        q = self.robotSystem.robotStateJointController.q
        stateMsg = robotstate.drakePoseToRobotState(q)
        stateMsg.utime = utimeUtil.getUtime()
        lcmUtils.publish("EST_ROBOT_STATE", stateMsg)

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()