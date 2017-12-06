#!/usr/bin/env python

from fusion_server.srv import *
import rospy

def handle_capture_fusion_data(req):
    print "Returning %s/2"%(req.a)
    return CaptureFusionDataResponse(req.a/2.0)

def capture_fusion_data_server():
    rospy.init_node('capture_fusion_data_server')
    s = rospy.Service('capture_fusion_data', CaptureFusionData, handle_capture_fusion_data)
    print "Ready to capture fusion data."
    rospy.spin()

if __name__ == "__main__":
    capture_fusion_data_server()