__author__ = 'manuelli'
import numpy as np
import collections
import yaml
import os
import datetime

# director
from director import transformUtils

import spartan.utils.transformations as transformations


def getSpartanSourceDir():
    return os.getenv("SPARTAN_SOURCE_DIR")

def getDictFromYamlFilename(filename):
    stream = file(filename)
    return yaml.load(stream)

def saveToYaml(data, filename):
    with open(filename, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)

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

def dictFromPosQuat(pos, quat):
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

def get_current_time_unique_name():
    """
    Converts current date to a unique name

    Note: this function will return variable-length strings.

    :return:
    :rtype: str
    """

    unique_name = time.strftime("%Y%m%d-%H%M%S")
    return unique_name

def homogenous_transform_from_dict(d):
    """
    Returns a transform from a standard encoding in dict format
    :param d:
    :return:
    """
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

    transform_matrix = transformations.quaternion_matrix(quat)
    transform_matrix[0:3,3] = np.array(pos)

    return transform_matrix


def get_current_YYYY_MM_DD_hh_mm_ss():
    """
    Returns a string identifying the current:
    - year, month, day, hour, minute, second

    Using this format:

    YYYY-MM-DD-hh-mm-ss

    For example:

    2018-04-07-19-02-50

    Note: this function will always return strings of the same length.

    :return: current time formatted as a string
    :rtype: string

    """

    now = datetime.datetime.now()
    string =  "%0.4d-%0.2d-%0.2d-%0.2d-%0.2d-%0.2d" % (now.year, now.month, now.day, now.hour, now.minute, now.second)
    return string
