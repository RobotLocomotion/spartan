#!/usr/bin/env python

import sys
import rospy
from fusion_server.srv import *

def start_bagging_fusion_data_client():
    print "Waiting for 'start_bagging_fusion_data' service..."
    rospy.wait_for_service('start_bagging_fusion_data')
    print "Found it!, starting bagging..."
    try:
        start_bagging_fusion_data = rospy.ServiceProxy('start_bagging_fusion_data', StartBaggingFusionData)
        resp1 = start_bagging_fusion_data()
        return resp1.data_filepath
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "don't need any args to start bagging!"

if __name__ == "__main__":
    if len(sys.argv) > 1:
        print usage()
        sys.exit(1)
    print "data_filepath = %s"%(start_bagging_fusion_data_client())