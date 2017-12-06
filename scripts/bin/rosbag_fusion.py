#!/usr/bin/env python

import os
import sys

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
for i in topics_to_bag:
	rosbag_cmd += " " + i

# add some visibility
print rosbag_cmd

# start bagging
# use Ctrl+C or otherwise end this process to stop
os.system("cd " + bagfile_directory + " && " + rosbag_cmd)