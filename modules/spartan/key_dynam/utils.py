#!/usr/bin/env python

import sys
import rospy
from imitation_tools.srv import *
import time


def start_bagging_imitation_data_client():
    print "Waiting for 'start_bagging_imitation_data' service..."
    rospy.wait_for_service('start_bagging_imitation_data', timeout=1.0)
    print "Found it!, starting bagging..."

    try:
        start_bagging_imitation_data = rospy.ServiceProxy('start_bagging_imitation_data', StartBaggingImitationData)
        resp1 = start_bagging_imitation_data()
        return resp1.bag_filepath
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def stop_bagging_imitation_data_client():

    try:
        print "Waiting for 'stop_bagging_imitation_data' service..."
        rospy.wait_for_service('stop_bagging_imitation_data', timeout=1.0)
        print "Found it!, stopping bagging..."
    except rospy.ROSException as e:
        print e
        return

    try:
        stop_bagging_imitation_data = rospy.ServiceProxy('stop_bagging_imitation_data', StopBaggingImitationData)
        resp1 = stop_bagging_imitation_data()
        return resp1.status
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e


def usage():
    return "don't need any args to start bagging!"


if __name__ == "__main__":
    if len(sys.argv) > 1:
        print usage()
        sys.exit(1)

    print "bag_filepath = %s" % (start_bagging_imitation_data_client())

    print "Started bagging in example client..."
    print "Now waiting a couple seconds before stopping..."
    time.sleep(5)
    print "Stopping.."

    print "bagging termination status = %s" % (stop_bagging_imitation_data_client())
