#!/usr/bin/env python

from fusion_server.srv import *
import rospy
import os
import sys
import time
import subprocess

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


class FusionServer(object): 

	def __init__(self):
		self.bagging = False

	def start_bagging(self):
		bagfile_directory = os.path.expanduser("~")+"/bagfiles/fusion/"

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
		rosbag_proc = subprocess.Popen(rosbag_cmd, stdin=subprocess.PIPE, shell=True, cwd=bagfile_directory)
		return os.path.join(bagfile_directory, bagfile_name+".bag"), rosbag_proc

	def handle_start_bagging_fusion_data(self, req):
		## check if bagging already
		if self.bagging:
			return StartBaggingFusionDataResponse("ERROR: Already bagging!")

		## start bagging
		filepath, rosbag_proc = self.start_bagging()

		## move to good positions for capturing fusion data
		time.sleep(2)

		## stop bagging 
		terminate_ros_node("/record")                       # this is heavier weight but will not create a .active
		# rosbag_proc.send_signal(subprocess.signal.SIGINT) # this is a more direct way of stopping the rosbag, but will terminate it with a .active

		## return the full path string to the data
		print "Returning filepath"
		return StartBaggingFusionDataResponse(filepath)

	def capture_fusion_data_server(self):
		rospy.init_node('capture_fusion_data_server')
		s = rospy.Service('start_bagging_fusion_data', StartBaggingFusionData, self.handle_start_bagging_fusion_data)
		print "Ready to capture fusion data."
		rospy.spin()

if __name__ == "__main__":
	fs = FusionServer()
	fs.capture_fusion_data_server()