#!/usr/bin/env python 

# Usage:
# source /opt/ros/kinetic/setup.bash
# ./publishCalibrationDataToVisp.py

import rospy
from geometry_msgs.msg import PoseStamped
from visp_hand2eye_calibration.msg import TransformArray
import yaml


###
# read in yaml file of camera poses and hand poses
###

###
# compute relative transforms for all
###

###
# publish transforms for visp
###

# init node
rospy.init_node('visp_calibration_data_publisher')
rospy.Publisher('/world_effector', TransformArray, queue_size=100)
rospy.Publisher('/camera_object', TransformArray, queue_size=100)


rospy.spin() # not actually necessary, just blocks