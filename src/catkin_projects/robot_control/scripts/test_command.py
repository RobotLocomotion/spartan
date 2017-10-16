#!/usr/bin/env python

import sys
import rospy
from robot_msgs.srv import *
from trajectory_msgs.msg import *


def command_interface(conf):
	# Get current conf

	# Convert conf to trajectory points

	rospy.wait_for_service('/robot_control/SendJointTrajectory')
	s = rospy.ServiceProxy('/robot_control/SendJointTrajectory', SendJointTrajectory)

	traj = JointTrajectory()

	# Append tajectory points


if __name__ == "__main__":
	rospy.wait_for_service('/robot_control/SendJointTrajectory')
	rospy.init_node('TrajectoryServer')
	s = rospy.ServiceProxy('robot_control/SendJointTrajectory', SendJointTrajectory)

	duration = 5 # seconds
	traj = JointTrajectory()
	p = JointTrajectoryPoint()


	traj.joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']

	p.positions = [0, 0, 0, 0, 0, 0, 0.0]
	p.velocities = [0, 0, 0, 0, 0, 0, 0.0]
	p.accelerations = [0, 0, 0, 0, 0, 0, 0.0]
	p.effort = [0, 0, 0, 0, 0, 0, 0.0]
	p.time_from_start = rospy.Duration(secs=3)
	traj.points.append(p)

	p2 = JointTrajectoryPoint()
	p2.positions = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0]
	p2.velocities = [0, 0, 0, 0, 0, 0, 0.0]
	p2.accelerations = [0, 0, 0, 0, 0, 0, 0.0]
	p2.effort = [0, 0, 0, 0, 0, 0, 0.0]
	p2.time_from_start = p.time_from_start + rospy.Duration(secs=3)
	traj.points.append(p2)

	p3 = JointTrajectoryPoint()
	p3.positions = [-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -1.0]
	p3.velocities = [0, 0, 0, 0, 0, 0, 0.0]
	p3.accelerations = [0, 0, 0, 0, 0, 0, 0.0]
	p3.effort = [0, 0, 0, 0, 0, 0, 0.0]
	p3.time_from_start = p2.time_from_start + rospy.Duration(secs=3)
	traj.points.append(p3)

	print 'Going'
	resp = s(traj, duration)
	print resp.success
	print 'Done!'
