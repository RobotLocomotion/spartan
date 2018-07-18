#! /usr/bin/env python
import yaml

# spartan
from robot_control.robotmovementservice import RobotMovementService
import spartan.utils.utils as spartan_utils

# ROS
import rospy


if __name__ == "__main__":
    rospy.init_node("RobotMovementService")
    config_filename = rospy.get_param("~config_filename")
    rospy.loginfo("config_filename: %s", config_filename)
    config = spartan_utils.getDictFromYamlFilename(config_filename)
    robotMovementService = RobotMovementService(config)
    robotMovementService.advertiseServices()
    rospy.loginfo("RobotMovementService ready!")
    while not rospy.is_shutdown():
        rospy.spin()