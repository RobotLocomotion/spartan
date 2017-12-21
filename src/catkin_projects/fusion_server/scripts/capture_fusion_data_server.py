#!/usr/bin/env python

#system
import rospy
import os
import sys
import time
import subprocess

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils

# ros srv
from fusion_server.srv import *


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
		self.rosbag_proc = None
		storedPosesFile = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects', 'station_config','RLG_iiwa_1','stored_poses.yaml')
		self.storedPoses = spartanUtils.getDictFromYamlFilename(storedPosesFile)
		self.robotService = rosUtils.RobotService(self.storedPoses['header']['joint_names'])
		self.setupConfig()

	def setupConfig(self):
		self.config = dict()
		self.config['scan'] = dict()
		self.config['scan']['pose_list'] = ['scan_back', 'scan_left', 'scan_top', 'scan_right', 'scan_back']
		self.config['scan']['joint_speed'] = 60
		self.config['home_pose_name'] = 'above_table_pre_grasp'

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
		filepath, self.rosbag_proc = self.start_bagging()
		self.bagging = True

		## return the full path string to the data
		print "Returning filepath"
		return StartBaggingFusionDataResponse(filepath)

	def handle_stop_bagging_fusion_data(self, req):
		## check if bagging already
		if not self.bagging:
			return StopBaggingFusionDataResponse("ERROR: Not currently bagging! Nothing to stop...")

		## stop bagging 
		terminate_ros_node("/record")                            # this is heavier weight but will not create a .active
		# self.rosbag_proc.send_signal(subprocess.signal.SIGINT) # this is a more direct way of stopping the rosbag, but will terminate it with a .active
		self.bagging = False

		return StopBaggingFusionDataResponse("success")

	def handle_capture_scene_and_fuse(self, req):
		# Start bagging with own srv call
		try:
			start_bagging_fusion_data = rospy.ServiceProxy('start_bagging_fusion_data', StartBaggingFusionData)
			resp1 = start_bagging_fusion_data()
			bag_filepath = resp1.data_filepath
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		# Move robot around
		for poseName in self.config['scan']['pose_list']:
			joint_positions = self.storedPoses[poseName]
			self.robotService.moveToJointPosition(joint_positions, maxJointDegreesPerSecond=self.config['scan']['joint_speed'])


		# Stop bagging with own srv call
		try:
			stop_bagging_fusion_data = rospy.ServiceProxy('stop_bagging_fusion_data', StopBaggingFusionData)
			resp1 = stop_bagging_fusion_data()
			print resp1.status
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		return CaptureSceneAndFuseResponse(bag_filepath)

	def run_fusion_data_server(self):
		rospy.init_node('capture_fusion_data_server')
		s = rospy.Service('start_bagging_fusion_data', StartBaggingFusionData, self.handle_start_bagging_fusion_data)
		s = rospy.Service('stop_bagging_fusion_data', StopBaggingFusionData, self.handle_stop_bagging_fusion_data)
		s = rospy.Service('capture_scene_and_fuse', CaptureSceneAndFuse, self.handle_capture_scene_and_fuse)
		print "Ready to capture fusion data."
		rospy.spin()

if __name__ == "__main__":
	fs = FusionServer()
	fs.run_fusion_data_server()