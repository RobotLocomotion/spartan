#!/usr/bin/env python

import sys
import rospy
import fusion_server
from fusion_server.srv import *
import time

def capture_scene_client():
    print "Waiting for 'capture_scene' service..."
    rospy.wait_for_service('capture_scene')
    print "Found it!, starting capture..."
    try:
        capture_scene = rospy.ServiceProxy('capture_scene', CaptureScene)
        resp = capture_scene()
        print "bag_filepath = %s" % resp.bag_filepath
        rospy.loginfo("bag_filepath = %s", resp.bag_filepath)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return resp

def usage():
    return "don't need any args to capture scene!"

if __name__ == "__main__":
    if len(sys.argv) > 1:
        print usage()
        sys.exit(1)
    
    capture_scene_client()
    
