#!/usr/bin/env python 

# Usage: (make sure to source ros's setup.bash in each terminal)
# roscore
#
# rosrun visp_hand2eye_calibratiosp_hand2eye_calibration_calibrator 
#
# ./publishCalibrationDataToVisp.py

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera

###
# read in yaml file of camera poses and hand poses
###

###
# compute relative transforms for all
###

###
# publish transforms for visp
###

# gather world transforms
world_effector_transforms = TransformArray()
world_effector_transforms.header.seq = 0
world_effector_transforms.header.stamp = 0
world_effector_transforms.header.frame_id = "world"

zero_transform = Transform()
for i in range(1000):
	world_effector_transforms.transforms.append(zero_transform)


# gather camera transforms
camera_object_transforms = TransformArray()
camera_object_transforms.header.seq = 0
camera_object_transforms.header.stamp = 0
camera_object_transforms.header.frame_id = "world"

zero_transform = Transform()
for i in range(1000):
	camera_object_transforms.transforms.append(zero_transform)


print len(world_effector_transforms.transforms)
print len(camera_object_transforms.transforms)

# init node and publishers
rospy.init_node('visp_calibration_data_publisher')
world_effector_pub = rospy.Publisher('/world_effector', TransformArray, queue_size=100)
camera_object_pub  = rospy.Publisher('/camera_object', TransformArray, queue_size=100)

# publish TransfromArrays

world_effector_pub.publish(world_effector_transforms)
camera_object_pub.publish(camera_object_transforms)

# initialize service proxy for the compute_effector_camera service
rospy.wait_for_service('/compute_effector_camera')
service_proxy = rospy.ServiceProxy('/compute_effector_camera', compute_effector_camera)
try:
  reply = service_proxy()
  print reply
except rospy.ServiceException as exc:
  print("Service did not process request: " + str(exc))


rospy.spin() # not actually necessary, just blocks