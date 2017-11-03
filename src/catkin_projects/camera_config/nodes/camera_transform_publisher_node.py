#!/usr/bin/env python

# ROS system imports
import rospy

# ROS custom imports
from camera_config.camera_transform_publisher import CameraTransformPublisher

if __name__=="__main__":

	rospy.init_node("camera_transform_publisher")
	CameraTransformPublisher = CameraTransformPublisher()

	rate = rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
	    CameraTransformPublisher.broadcastTransforms()
	    rate.sleep()
	