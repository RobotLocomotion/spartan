#!/usr/bin/env python

import argparse

# ROS
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2


def getSingleImage(topic, encoding=None):
    msgType = sensor_msgs.msg.Image
    rospy.loginfo("waiting for image on topic %s", topic)
    bridge = CvBridge()
    d = dict()
    msg = rospy.wait_for_message(topic, msgType)
    rospy.loginfo("received message on topic %s", topic)
    print "type(msg) ", type(msg)

    if encoding is None:
        encoding = msg.encoding

    print "encoding = ", encoding

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, encoding)
    except CvBridgeError, e:
        print(e)

    d['msg'] = msg
    d['cv2_img'] = cv2_img

    rospy.loginfo("converted msg to cv2 img on topic %s", topic)

    return d


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic", type=str, required=True, help="name of the topic to record")
    parser.add_argument("-f", "--filename", type=str, required=True, help="filename to which to save the image")
    parser.add_argument("-e", "--encoding", type=str, required=False, help="encoding type for CvBridge.imgmsg_to_cv2")

    args = parser.parse_args()
    rospy.init_node("image_capture")

    data = getSingleImage(args.topic, encoding=args.encoding)
    rospy.loginfo("writing image to file %s", args.filename)
    cv2.imwrite(args.filename, data['cv2_img'])
    rospy.loginfo("finished writing image to file")

