__author__ = 'manuelli'
import numpy as np
import collections
import yaml
import os

# director
from director import transformUtils


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


