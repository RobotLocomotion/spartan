#!/usr/bin/env python

# Example usage:
# python simple_subscriber.py --topic-package-type ['/camera/rgb/image_raw','sensor_msgs.msg','Image']


import argparse
import importlib

# ROS
import rospy

import spartan.utils.ros_utils as spartanROSUtils


def getMessageType(packageName, msgName):
    pkg = importlib.import_module(packageName)
    msg = getattr(pkg, msgName)
    return msg


def makeSubscriber(topic, packageName, msgName):
    msgType = getMessageType(packageName, msgName)
    subscriber = spartanROSUtils.SimpleSubscriber(topic, msgType)
    subscriber.start()
    return subscriber


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic-package-type', type=str, nargs='+')
    args = parser.parse_args()
    
    for i in args.topic_package_type:
        print i
        topic_package_type = i.strip("[]").split(',')
        topic = topic_package_type[0]
        package = topic_package_type[1]
        msg_type = topic_package_type[2]
        makeSubscriber(topic, package, msg_type)

    rospy.init_node("simple_subscriber", anonymous=True)
    rospy.spin()



