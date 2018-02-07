#!/usr/bin/env python
import sys
import os
import rospy
import numpy as np
import time

import sensor_msgs.msg
from spartan.utils.ros_utils import *
from spartan.utils.utils import poseFromTransform
from tf import TransformListener
import geometry_msgs
import robot_msgs.srv
import perception_sandbox.srv
import perception_sandbox.msg
import spartan_grasp_msgs
import sensor_msgs
import std_srvs.srv
from director import transformUtils

from collect_point_clouds_with_arm import PointCloudObjectFittingServer

def cleanup_and_exit(code):
    sys.exit(code)

if __name__=="__main__":
    rospy.init_node("collect_point_clouds_with_arm_object_fit", anonymous=True)

    meshes = ["${SPARTAN_SOURCE_DIR}/models/rlg_misc_models/meshes/visual/companion_cube.obj"]
    meshes = [os.path.expandvars(mesh) for mesh in meshes]

    pcserver = PointCloudObjectFittingServer()
    print pcserver.FitObjectsByIcp(meshes, [0.003])