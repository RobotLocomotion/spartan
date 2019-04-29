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

import ros_numpy

import spartan.utils.utils as spartan_utils


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

    quatDict = getQuaternionFromDict(d)
    quat = [0]*4
    quat[0] = quatDict['w']
    quat[1] = quatDict['x']
    quat[2] = quatDict['y']
    quat[3] = quatDict['z']

    return transformUtils.transformFromPose(pos, quat)

"""
msg: geometry_msgs/Pose
"""
def transformFromROSPoseMsg(msg):
    pos = [msg.position.x, msg.position.y, msg.position.z]
    quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

    return transformUtils.transformFromPose(pos,quat)

def transformFromROSTransformMsg(msg):
    pos = [msg.translation.x, msg.translation.y, msg.translation.z]
    quat = [msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z]

    return transformUtils.transformFromPose(pos,quat)

def getQuaternionFromDict(d):
    quat = None
    quatNames = ['orientation', 'rotation', 'quaternion']
    for name in quatNames:
        if name in d:
            quat = d[name]


    if quat is None:
        raise ValueError("Error when trying to extract quaternion from dict, your dict doesn't contain a key in ['orientation', 'rotation', 'quaternion']")

    return quat


def numpy_from_pointcloud2_msg(msg):
    """

    :param msg: sensor_msgs/PointCloud2
    :type msg:
    :return:
    :rtype:
    """
    pc = ros_numpy.numpify(msg)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    return points

def save_transform_to_file(transform):
    filename = os.path.join(spartan_utils.get_sandbox_dir(), "transform.yaml")
    d = poseFromTransform(transform)
    spartan_utils.saveToYaml(d, filename)
    print("saved transform to %s" %(filename))



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
