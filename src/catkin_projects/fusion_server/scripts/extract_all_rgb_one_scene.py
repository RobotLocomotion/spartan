#!/usr/bin/python

import os
import rospy
import time
import gc
import sys

from fusion_server.fusion import FusionServer
import fusion_server.tsdf_fusion as tsdf_fusion

import spartan.utils.utils as spartanUtils


if __name__ == "__main__":

    bag_filepath = sys.argv[1]

    print "using bag_full_path", bag_filepath

    start = time.time()

    fs = FusionServer()
    
    processed_dir, images_dir = fs.extract_data_from_rosbag(bag_filepath, rgb_only=True, save_folder_name='rgb_images_only')

    end = time.time()
    hours, rem = divmod(end-start, 3600)
    minutes, seconds = divmod(rem, 60)
    time_string = "{:0>2}:{:0>2}:{:05.2f}".format(int(hours),int(minutes),seconds)
    print "total time:               ",  time_string
    print "(hours, minutes, seconds with two decimals)"





