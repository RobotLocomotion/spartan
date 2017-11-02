#!/usr/bin/env python

# ROS system imports
import rospy

# ROS custom imports
from camera_config.camera_info_publisher import CameraInfoPublisher

if __name__=="__main__":

	rospy.init_node("camera_info_publisher")
	cameraInfoPublisher = CameraInfoPublisher()

	rate = rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
	    cameraInfoPublisher.broadcastTransforms()
	    rate.sleep()
	