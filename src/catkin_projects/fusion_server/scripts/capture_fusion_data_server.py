#!/usr/bin/env python

from fusion_server.srv import *
import rospy
import os
import sys
import time
import subprocess

def start_bagging():
	bagfile_directory = "/home/peteflo/bagfiles/fusion/"

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
	#os.system("cd " + bagfile_directory + " && timeout 2s " + rosbag_cmd + " &")
	rosbag_proc = subprocess.Popen(rosbag_cmd, stdin=subprocess.PIPE, shell=True, cwd=bagfile_directory)
	return os.path.join(bagfile_directory, bagfile_name), rosbag_proc

# this function taken from here:
# https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)

def handle_capture_fusion_data(req):

	## start bagging
	filepath, rosbag_proc = start_bagging()

	## move to good positions for capturing fusion data
	time.sleep(2)

	## stop bagging -- this is heavier weight but will not create a .active
	terminate_ros_node("/record")

	## stop bagging -- this is a more direct way of stopping the rosbag, but will terminate it with a .active
	# rosbag_proc.send_signal(subprocess.signal.SIGINT)

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