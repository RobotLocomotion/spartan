#!/usr/bin/env python

import sys

# ROS
import rospy
import sensor_msgs.msg


# ROS custom
import robot_msgs.srv
import robot_control.control_utils as controlUtils



def moveToJointPosition():
    maxJointDegreesPerSecond = 30
    jointPosition = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0]

    jointState = sensor_msgs.msg.JointState()
    jointState.header.stamp = rospy.Time.now()


    jointState.position = jointPosition
    jointState.name = controlUtils.getIiwaJointNames()

    numJoints = len(jointState.name)
    jointState.velocity = [0]*numJoints
    jointState.effort = [0]*numJoints

    rospy.wait_for_service('robot_control/MoveToJointPosition')
    s = rospy.ServiceProxy('robot_control/MoveToJointPosition', robot_msgs.srv.MoveToJointPosition)
    response = s(jointState, maxJointDegreesPerSecond)

    print "response ", response



if __name__ == "__main__":
	rospy.init_node('TestMovementService')
	moveToJointPosition()
