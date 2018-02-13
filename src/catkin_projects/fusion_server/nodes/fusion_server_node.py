#!/usr/bin/python
import rospy
from fusion_server.fusion import FusionServer

if __name__ == "__main__":
    rospy.init_node("fusion_server")
    camera_serial_number = rospy.get_param('~camera_serial_number')
    fs = FusionServer(camera_serial_number=camera_serial_number)
    fs.run_fusion_data_server()