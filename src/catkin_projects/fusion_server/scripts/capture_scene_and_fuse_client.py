#!/usr/bin/env python

import sys
import rospy
from fusion_server.srv import *
import time

def capture_scene_and_fuse_client():
    print "Waiting for 'capture_scene_and_fuse' service..."
    rospy.wait_for_service('capture_scene_and_fuse')
    print "Found it!, starting capture..."
    try:
        capture_scene_and_fuse = rospy.ServiceProxy('capture_scene_and_fuse', CaptureSceneAndFuse)
        resp1 = capture_scene_and_fuse()
        return resp1.pointcloud_filepath
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "don't need any args to capture scene!"

if __name__ == "__main__":
    if len(sys.argv) > 1:
        print usage()
        sys.exit(1)
    
    print "data_filepath = %s"%(capture_scene_and_fuse_client())
