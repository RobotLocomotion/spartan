#!/usr/bin/env python

import sys
import rospy
from fusion_server.srv import *
import time

def capture_scene_for_fusion_client():
    print "Waiting for 'capture_scene_for_fusion' service..."
    rospy.wait_for_service('capture_scene_for_fusion')
    print "Found it!, starting capture..."
    try:
        capture_scene_for_fusion = rospy.ServiceProxy('capture_scene_for_fusion', CaptureSceneForFusion)
        resp1 = capture_scene_for_fusion()
        return resp1.data_filepath
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "don't need any args to capture scene!"

if __name__ == "__main__":
    if len(sys.argv) > 1:
        print usage()
        sys.exit(1)
    
    print "data_filepath = %s"%(capture_scene_for_fusion_client())
