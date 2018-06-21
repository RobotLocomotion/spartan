#!/usr/bin/env python

import sys

# ROS
import rospy
import sensor_msgs.msg
import tf2_ros


# ROS custom
import geometry_msgs.msg
import robot_msgs.srv
import robot_control.control_utils as controlUtils

# spartan
import spartan.utils.ros_utils as rosUtils

above_table_pre_grasp = [0.04486168762069299, 0.3256606458812486, -0.033502080520812445, -1.5769091802934694, 0.05899249087322813, 1.246379583616529, 0.38912999977004026]

def testIkService():
    maxJointDegreesPerSecond = 30
    poseStamped = geometry_msgs.msg.PoseStamped()

    # poseStamped.pose.position.x = 2.00213459e-01
    # poseStamped.pose.position.y = -1.93298822e-12
    # poseStamped.pose.position.z = 8.99913227e-01

    # poseStamped.pose.orientation.w = 9.99999938e-01
    # poseStamped.pose.orientation.x = -1.16816462e-12
    # poseStamped.pose.orientation.y = 3.51808464e-04
    # poseStamped.pose.orientation.z = -5.36917849e-12

    pos = [ 0.51003723,  0.02411757,  0.30524811]
    quat = [ 0.68763689,  0.15390449,  0.69872774, -0.12348466]

    poseStamped.pose.position.x = pos[0]
    poseStamped.pose.position.y = pos[1]
    poseStamped.pose.position.z = pos[2]

    poseStamped.pose.orientation.w = quat[0]
    poseStamped.pose.orientation.x = quat[1]
    poseStamped.pose.orientation.y = quat[2]
    poseStamped.pose.orientation.z = quat[3]

    robotService = rosUtils.RobotService.makeKukaRobotService()
    response = robotService.runIK(poseStamped, seedPose=above_table_pre_grasp, nominalPose=above_table_pre_grasp)

    
    print "IK solution found ", response.success
    

    if response.success:
        print "moving to joint position", response.joint_state.position
        robotService.moveToJointPosition(response.joint_state.position)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    ee_frame_name = "iiwa_link_ee"
    world_frame_name = "base"
    iiwa_link_ee_to_world = tfBuffer.lookup_transform(world_frame_name, ee_frame_name, rospy.Time(0), rospy.Duration(1.0))

    print "iiwa_link_ee_to_world", iiwa_link_ee_to_world

if __name__ == "__main__":
	rospy.init_node('TestMovementService')
	testIkService()
