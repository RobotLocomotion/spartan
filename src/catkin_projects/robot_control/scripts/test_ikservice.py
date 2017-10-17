#!/usr/bin/env python

import sys

# ROS
import rospy
import sensor_msgs.msg


# ROS custom
import geometry_msgs.msg
import robot_msgs.srv
import robot_control.control_utils as controlUtils



def testIkService():
    maxJointDegreesPerSecond = 30
    poseStamped = geometry_msgs.msg.PoseStamped()

    poseStamped.pose.position.x = 2.00213459e-01
    poseStamped.pose.position.y = -1.93298822e-12
    poseStamped.pose.position.z = 8.99913227e-01

    poseStamped.pose.orientation.w = 9.99999938e-01
    poseStamped.pose.orientation.x = -1.16816462e-12
    poseStamped.pose.orientation.y = 3.51808464e-04
    poseStamped.pose.orientation.z = -5.36917849e-12

    rospy.wait_for_service('robot_control/IkService')
    ikService = rospy.ServiceProxy('robot_control/IkService', robot_msgs.srv.RunIK)
    response = ikService(poseStamped)
    print "IK solution found ", response.success

    if response.success:
        rospy.wait_for_service('robot_control/MoveToJointPosition')
        moveService = rospy.ServiceProxy('robot_control/MoveToJointPosition', robot_msgs.srv.MoveToJointPosition)


        rospy.loginfo("executing plan")
        response = moveService(response.joint_state, maxJointDegreesPerSecond)
        rospy.loginfo("plan finished")




if __name__ == "__main__":
	rospy.init_node('TestMovementService')
	testIkService()
