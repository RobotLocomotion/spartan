#!/usr/bin/env python

from fusion_server.srv import *
import rospy
import os
import sys
import time

def start_bagging():
	bagfile_directory = "~/bagfiles/fusion/"

	# make sure bagfile_directory exists
	os.system("mkdir -p " + bagfile_directory)

	# camera serial number
	camera_serial_number = "1112170110"

	topics_to_bag = [
		"/tf",
		"/tf_static",
		"/camera_"+camera_serial_number+"/depth_registered/sw_registered/image_rect",
		"/camera_"+camera_serial_number+"/depth_registered/sw_registered/camera_info",
		"/camera_"+camera_serial_number+"/rgb/camera_info",
		"/camera_"+camera_serial_number+"/rgb/image_rect_color"
	]

	# build up command string
	rosbag_cmd = "rosbag record"
	bagfile_name = "fusion" + str(time.time())
	rosbag_cmd += " -O " + bagfile_name
	for i in topics_to_bag:
		rosbag_cmd += " " + i

	# add some visibility
	print rosbag_cmd

	# start bagging
	# for now, timeout the bag
	os.system("cd " + bagfile_directory + " && timeout 2s " + rosbag_cmd + " &")
	return os.path.join(bagfile_directory, bagfile_name)

def handle_capture_fusion_data(req):

	## start bagging
	filepath = start_bagging()

	## move to good positions for capturing fusion data

	## return the full path string to the data
	print "Returning filepath"
	return CaptureFusionDataResponse(filepath)

def capture_fusion_data_server():
	rospy.init_node('capture_fusion_data_server')
	s = rospy.Service('capture_fusion_data', CaptureFusionData, handle_capture_fusion_data)
	print "Ready to capture fusion data."
	rospy.spin()

if __name__ == "__main__":
	capture_fusion_data_server()