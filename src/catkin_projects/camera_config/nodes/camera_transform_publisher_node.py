#!/usr/bin/env python

# ROS system imports
import rospy

# ROS custom imports
from camera_config.camera_transform_publisher import CameraTransformPublisher

# Avoid name colliding
import random

if __name__=="__main__":
	node_name = "camera_transform_publisher" + str(random.randint(0, 100000000))
	rospy.init_node(node_name)
	CameraTransformPublisher = CameraTransformPublisher()

	rate = rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
	    CameraTransformPublisher.broadcastTransforms()
	    rate.sleep()
	
