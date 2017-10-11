#!/usr/bin/env python 

# Usage: (make sure to source ros's setup.bash in each terminal)
# roscore
#
# rosrun visp_hand2eye_calibratiosp_hand2eye_calibration_calibrator 
#
# directorPython publishCalibrationDataToVisp.py

import rospy
import yaml
import time
import random

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera

sys.path.append('../LabelFusion/modules/labelfusion')
from cameraposes import CameraPoses

###
# read in yaml file of camera poses and hand poses
###

yaml_file_name = "./data/20171010-173517/robot_data.yaml"
with open(yaml_file_name, 'r') as f:
    calib_data = yaml.load(f)

for i in range(len(calib_data)):
	print i
	print calib_data[i]

###
# compute relative transforms for all
###

###
# publish transforms for visp
###

def randomTransform():
	transform = Transform()
	transform.translation.x = random.uniform(-1,1)
	transform.translation.y = random.uniform(-1,1)
	transform.translation.z = random.uniform(-1,1)
	transform.rotation.x    = random.uniform(-1,1)
	transform.rotation.y    = random.uniform(-1,1)
	transform.rotation.z    = random.uniform(-1,1)
	transform.rotation.w    = 1.0
	return transform

def applyCameraTransform(transform_in):
	transform = Transform()
	transform.translation.x = transform_in.translation.x
	transform.translation.y = transform_in.translation.y
	transform.translation.z = transform_in.translation.z + 0.5
	transform.rotation.x    = transform_in.rotation.x   
	transform.rotation.y    = transform_in.rotation.y   
	transform.rotation.z    = transform_in.rotation.z   
	transform.rotation.w    = transform_in.rotation.w   
	return transform

# gather world transforms
world_effector_transforms = TransformArray()
world_effector_transforms.header.seq = 0
world_effector_transforms.header.stamp = 0
world_effector_transforms.header.frame_id = "world"

for i in range(10):
	world_effector_transforms.transforms.append(randomTransform())

# gather camera transforms
camera_object_transforms = TransformArray()
camera_object_transforms.header.seq = 0
camera_object_transforms.header.stamp = 0
camera_object_transforms.header.frame_id = "world"

for i,v in enumerate(world_effector_transforms.transforms):
	camera_object_transforms.transforms.append(applyCameraTransform(v))

# init node and publishers
rospy.init_node('visp_calibration_data_publisher')
world_effector_pub = rospy.Publisher('/world_effector', Transform, queue_size=1000)
camera_object_pub  = rospy.Publisher('/camera_object', Transform, queue_size=1000)

# hack: need to sleep between advertising topic and publishing topic, otherwise
# visp may not receive all of them
time.sleep(2)

# publish Transfroms

for i in range(len(world_effector_transforms.transforms)):
	world_effector_pub.publish(world_effector_transforms.transforms[i])
	camera_object_pub.publish(camera_object_transforms.transforms[i])

# hack: need to sleep after publishing all transforms, otherwise visp may not
# be ready when it is asked to compute the hand-eye ("effector_camera") transform
time.sleep(2)

# initialize service proxy for the compute_effector_camera service
rospy.wait_for_service('/compute_effector_camera')
service_proxy = rospy.ServiceProxy('/compute_effector_camera', compute_effector_camera)
try:
  reply = service_proxy()
  print reply
except rospy.ServiceException as exc:
  print("Service did not process request: " + str(exc))

rospy.spin() # not actually necessary, just blocks