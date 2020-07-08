__author__ = 'manuelli'


import numpy as np
import random
import json
import pickle
from scipy.spatial.transform import Rotation
import collections
import yaml
from yaml import CLoader
import os
import datetime

# director
from director import transformUtils

import spartan.utils.transformations as transformations


def getSpartanSourceDir():
    return os.getenv("SPARTAN_SOURCE_DIR")


def get_data_dir():
    return os.getenv("DATA_DIR")


def getDictFromYamlFilename(filename):
    """
    Read data from a YAML files
    """
    return yaml.load(file(filename), Loader=CLoader)

def saveToYaml(data, filename):
    with open(filename, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)

def save_to_json(data, filename):
    with open(filename, 'w') as outfile:
        json.dump(data, outfile)

def save_pickle(data, filename):
    with open(filename, 'wb') as f:
        pickle.dump(data, f)

def load_pickle(filename):
    with open(filename, 'rb') as f:
        data = pickle.load(f)
    return data

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

def transformFromROSPoseMsg(msg):
    pos = [msg.position.x, msg.position.y, msg.position.z]
    quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

    return transformUtils.transformFromPose(pos,quat)

def transformFromROSTransformMsg(msg):
    pos = [msg.translation.x, msg.translation.y, msg.translation.z]
    quat = [msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z]

    return transformUtils.transformFromPose(pos,quat)

def invert_transform(transform4):
    transform4_copy = np.copy(transform4)
    R = transform4_copy[0:3,0:3]
    R = np.transpose(R)
    transform4_copy[0:3,0:3] = R
    t = transform4_copy[0:3,3]
    inv_t = -1.0 * np.transpose(R).dot(t)
    transform4_copy[0:3,3] = inv_t
    return transform4_copy

def getQuaternionFromDict(d):
    quat = None
    quatNames = ['orientation', 'rotation', 'quaternion']
    for name in quatNames:
        if name in d:
            quat = d[name]


    if quat is None:
        raise ValueError("Error when trying to extract quaternion from dict, your dict doesn't contain a key in ['orientation', 'rotation', 'quaternion']")

    return quat

def get_translation_from_dict(d):
    translation = None
    names = ['position', 'translation']
    for name in names:
        if name in d:
            translation = d[name]


    if translation is None:
        raise ValueError("Error when trying to extract translation from dict, your dict doesn't contain a key in ['translation', 'position']")

    return translation

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
    pos_dict = get_translation_from_dict(d)
    pos = [0]*3
    pos[0] = pos_dict['x']
    pos[1] = pos_dict['y']
    pos[2] = pos_dict['z']

    quatDict = getQuaternionFromDict(d)
    quat = [0]*4
    quat[0] = quatDict['w']
    quat[1] = quatDict['x']
    quat[2] = quatDict['y']
    quat[3] = quatDict['z']

    transform_matrix = transformations.quaternion_matrix(quat)
    transform_matrix[0:3,3] = np.array(pos)

    return transform_matrix

def dict_from_homogenous_transform(tf):
    """
    Returns standard encoding in dict format from 4x4 transform matrix
    :param tf:
    :return:
    """
    return dictFromPosQuat(tf[:3, 3], transformations.quaternion_from_matrix(tf))

def apply_homogenous_transform_to_points(tf, pts):
    ''' Given a homogenous tf matrix and a 3xN NP array
    of points, apply the tf to the points to produce
    a new 3xN array of points.
    :param tf: 4x4 numpy array of matching dtype to pts
    :param pts: 3xN numpy array of matching dtype to tf
    :return: 3xN numpy array of matching dtype to tf and pts'''
    return ((tf[:3, :3].dot(pts).T) + tf[:3, 3]).T

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

def compute_angle_between_quaternions(q, r):
    """
    Computes the angle between two quaternions.

    theta = arccos(2 * <q1, q2>^2 - 1)

    See https://math.stackexchange.com/questions/90081/quaternion-distance
    :param q: numpy array in form [w,x,y,z]. As long as both q,r are consistent it doesn't matter
    :type q:
    :param r:
    :type r:
    :return: angle between the quaternions, in radians
    :rtype:
    """

    theta = 2*np.arccos(2 * np.dot(q,r)**2 - 1)
    return theta


def angle_axis_from_rotation_matrix(R):
    """
    Compute angle-axis representation from rotation matrix
    :param R:
    :type R:
    :return:
    :rtype:
    """
    # lowercase denotes scipy Rotation objects
    r = Rotation.from_dcm(R)
    return r.as_rotvec()

def rotation_matrix_from_angle_axis(rotvec):
    r = Rotation.from_rotvec(rotvec)
    return r.as_dcm()

def compute_translation_distance_between_poses(pose_a, pose_b):
    """
    Computes the linear difference between pose_a and pose_b
    :param pose_a: 4 x 4 homogeneous transform
    :type pose_a:
    :param pose_b:
    :type pose_b:
    :return: Distance between translation component of the poses
    :rtype:
    """

    pos_a = pose_a[0:3,3]
    pos_b = pose_b[0:3,3]

    return np.linalg.norm(pos_a - pos_b)

def compute_angle_between_poses(pose_a, pose_b):
    """
    Computes the angle distance in radians between two homogenous transforms
    :param pose_a: 4 x 4 homogeneous transform
    :type pose_a:
    :param pose_b:
    :type pose_b:
    :return: Angle between poses in radians
    :rtype:
    """

    quat_a = transformations.quaternion_from_matrix(pose_a)
    quat_b = transformations.quaternion_from_matrix(pose_b)

    return compute_angle_between_quaternions(quat_a, quat_b)


def get_kuka_joint_names():
    return [
     'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',
     'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6',
     'iiwa_joint_7']


def set_random_seed(seed):
    np.random.seed(seed)
    random.seed(seed)
