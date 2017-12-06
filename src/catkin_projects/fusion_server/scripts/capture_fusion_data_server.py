#!/usr/bin/env python

from fusion_server.srv import *
import rospy

def handle_capture_fusion_data(req):

	## start bagging

	## move to good positions for capturing fusion data

	## return the full path string to the data

    print "Returning filepath"
    return CaptureFusionDataResponse("/home/peteflo/data")

def capture_fusion_data_server():
    rospy.init_node('capture_fusion_data_server')
    s = rospy.Service('capture_fusion_data', CaptureFusionData, handle_capture_fusion_data)
    print "Ready to capture fusion data."
    rospy.spin()

if __name__ == "__main__":
    capture_fusion_data_server()