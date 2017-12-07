#!/usr/bin/env python

# system
import argparse
import numpy as np

# ROS
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2

# spartan
import spartan.utils.ros_utils as rosUtils


def getSingleImage(topic, encoding=None):
    msgType = sensor_msgs.msg.Image
    rospy.loginfo("waiting for image on topic %s", topic)
    bridge = CvBridge()
    d = dict()
    msg = rospy.wait_for_message(topic, msgType)
    rospy.loginfo("received message on topic %s", topic)
    print "type(msg) ", type(msg)
    print "encoding ", msg.encoding

    if encoding is None:
        print "using passthrough encoding"
        cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    else:
        print "encoding = ", encoding

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, encoding)
        except CvBridgeError, e:
            print(e)

    

    print "type(cv2_img) = ", type(cv2_img)
    print "cv2_img.dtype = ", cv2_img.dtype 

    # do some conversion if it's a depth image
    cv2_img_copy = None
    if ((cv2_img.dtype == np.float32) or (cv2_img.dtype == np.float64)):
        print "got a pointcloud, doing conversion from float to int"
        print "flags = ", cv2_img.flags
        # cv2_img.setflags(write=1)
        cv2_img = rosUtils.convert32FCto16UC(cv2_img)
        print "cv2_img.dtype = ", cv2_img.dtype
        # print cv2_img[200:210, 200:210]
        # print cv2_img

    

    rospy.loginfo("converted msg to cv2 img on topic %s", topic)

    d['msg'] = msg
    d['cv2_img'] = cv2_img
    return d


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic", type=str, required=True, help="name of the topic to record")
    parser.add_argument("-f", "--filename", type=str, required=True, help="filename to which to save the image")
    parser.add_argument("-e", "--encoding", type=str, required=False, help="encoding type for CvBridge.imgmsg_to_cv2")

    parser.add_argument("-fs", "--filestorage", type=bool, required=False, help="use the opencv FileStorage class for saving the file, filename must be of type yaml or xml")

    args = parser.parse_args()
    rospy.init_node("image_capture")

    data = getSingleImage(args.topic, encoding=args.encoding)
    rospy.loginfo("writing image to file %s", args.filename)

    if not args.filestorage:
        cv2.imwrite(args.filename, data['cv2_img'])
    else:
        fs_write = cv2.FileStorage(filename, cv2.FILE_STORAGE_WRITE)
        fs_write.write("data", d['cv2_img'])


    rospy.loginfo("finished writing image to file")

