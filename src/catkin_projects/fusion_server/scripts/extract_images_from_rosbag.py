#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("encoding", help="Encoding, for example bgr8.")
    parser.add_argument("save_time", help="Bool whether or not you want to save a file for the time")

    args = parser.parse_args()

    if "rgb" in args.image_topic:
        image_type = "rgb"
    elif "depth" in args.image_topic:
        image_type = "depth"
    else:
        raise ValueError("I don't know this image type.")

    print "Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir)

    bag = rosbag.Bag(args.bag_file, "r")
    print bag
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        print topic
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding=args.encoding)
        cv2.imwrite(os.path.join(args.output_dir, "%06i_%s.png" % (count,image_type)), cv_img)
        print "Wrote image %i" % count

        if args.save_time:
            #print "time is  ", msg.header.stamp#.to_nsec()*1.0/1e6
            #print "to_sec is", msg.header.stamp.to_sec()
            #print "to_nsc is", msg.header.stamp.to_nsec()
            nano_secs =  msg.header.stamp.to_nsec()
            milli_secs = int(str(nano_secs)[0:-6])
            decimal    = int(str(nano_secs)[-6:])
            time_out_file = "%06i_%s.txt" % (count,"millisecs")
            time_out_file_fullpath = os.path.join(args.output_dir, time_out_file)
            os.system("echo "+str(milli_secs)+"."+str(decimal)+" > "+time_out_file_fullpath)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()