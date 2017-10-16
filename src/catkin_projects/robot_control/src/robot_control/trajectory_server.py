#!/usr/bin/env python

# system
import os
import copy
import numpy as np
import threading
import time
import math

# drake
import lcm
from drake import lcmt_iiwa_command, lcmt_iiwa_status ,lcmt_robot_state
import pydrake
from pydrake.solvers import ik
from robotlocomotion import robot_plan_t
from bot_core import robot_state_t

# ROS
import rospy

# ROS custom packages
from robot_msgs.msg import *
from robot_msgs.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_control.control_utils import wait_for_convergence, update_conf_state


class TrajectoryServer:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.lc = lcm.LCM()
        self.publishChannel = "COMMITTED_ROBOT_PLAN"

    def handle_send_trajectory(self, req):
        resp = SendJointTrajectoryResponse()
        plan = TrajectoryServer.ROSJointTrajectoryToLCMRobotPlan(req.trajectory)
        planDuration = req.trajectory.points[-1].time_from_start

        self.lc.publish(self.publishChannel, plan.encode())
        eps_nsecs = int((0.2)*1e9)
        rospy.sleep(planDuration + rospy.Duration(nsecs=eps_nsecs))

        resp.success = True
        return resp

    def spinOnce(self):
        self.lc.handle()
        self.robot_conf_state = update_conf_state(self.robot_conf_state, self.joint_positions, self.command_positions, isDict=False)

    def advertiseService(self):
        rospy.Service('/robot_control/SendJointTrajectory', SendJointTrajectory, self.handle_send_trajectory)

    @staticmethod
    def ROSJointTrajectoryToLCMRobotPlan(jointTrajectoryRos):
        plan = robot_plan_t()
        plan.utime = 0
        plan.robot_name = ""
        plan.num_states = len(jointTrajectoryRos.points)
        plan.plan_info = [1]*plan.num_states

        jointNames = jointTrajectoryRos.joint_names
        for idx, jointTrajectoryPoint in enumerate(jointTrajectoryRos.points):
            robotState = TrajectoryServer.ROSJointTrajectoryPointToLCMRobotState(jointNames, jointTrajectoryPoint)

            plan.plan.append(robotState)


        return plan

    @staticmethod
    def ROSJointTrajectoryPointToLCMRobotState(jointNames, jointTrajectoryPoint):
        robotState = robot_state_t()
        robotState.num_joints = len(jointTrajectoryPoint.positions)
        robotState.joint_name = jointNames

        # print "duration = ", jointTrajectoryPoint.time_from_start.to_sec()
        # print "typeof(duration) ", typeof(JointTrajectoryPoint.time_from_start)

        robotState.utime = TrajectoryServer.ROSdurationToUtime(jointTrajectoryPoint.time_from_start)



        robotState.joint_position = jointTrajectoryPoint.positions
        robotState.joint_velocity = jointTrajectoryPoint.velocities
        robotState.joint_effort = jointTrajectoryPoint.effort

        return robotState

    @staticmethod
    def ROSdurationToUtime(duration):
        return int(duration.to_sec()*1e6)



if __name__ == '__main__':
    rospy.init_node('TrajectoryServer')
    fs = TrajectoryServer()
    print "Starting TrajectoryServer"
    fs.advertiseService()
    while not rospy.is_shutdown():
        fs.spinOnce()
        fs.rate.sleep()
