#!/usr/bin/env python

import os, copy, numpy as np, threading, lcm
from drake import lcmt_iiwa_command, lcmt_iiwa_status ,lcmt_robot_state
import rospy, time, math
import pydrake
from pydrake.solvers import ik
from robotlocomotion import robot_plan_t
from bot_core import robot_state_t
from robot_msgs.msg import *
from robot_msgs.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_control.control_utils import wait_for_convergence, update_conf_state

max_trajectory_time = rospy.get_param('trajectory_time')

class TrajectoryServer:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.lc = lcm.LCM()

        iiwa_status_sub = self.lc.subscribe('IIWA_STATUS', self.iiwa_status_sub)
        self.joint_positions = None
        self.command_positions = None
        self.robot_conf_state = 'converged'


    def trajectory_runner(self, joint_trajectory, duration):
        self.lc.publish("CANCEL_PLAN",lcmt_iiwa_status().encode() )

        if duration <= 0.0: duration = max_trajectory_time
        plan = robot_plan_t()
        plan.utime = 0.0
        plan.robot_name = "lbr_iiwa_14"
        plan.num_states = len(joint_trajectory.points)
        print plan.num_states
        plan.plan_info = [1]*len(joint_trajectory.points)

        p = joint_trajectory.points
        self.command_positions = self.joint_positions

        # Attempt to compute the overall move distance
        joint_positions, joint_deltas = [], []
        for i in xrange(0, len(p)):
            joint_positions.append(p[i].positions)
            if i > 0:
                delta = tuple(map(lambda x, y: abs(x - y), joint_positions[i], joint_positions[i-1]))
                joint_deltas.append(delta)

        # !! Jay, attempt to weigh the overall time based on each joint
        overall_dist = np.sum(joint_deltas, axis=0)
        print overall_dist
        multiplier = [1.25, 1.15, 1.15, 1.0, 1.0, 0.75, 0.25]
        overall_time= [a*b for a,b in zip(overall_dist, multiplier)]
        print overall_time

        duration = min(max(np.sum(overall_time), 1.25), 8.75)

        print '[robot_control]: Attempting to use the move time of', duration
        # raw_input('Continue?')

        for i in xrange(0, len(p)):
            s = robot_state_t()
            s.utime = i * (duration * 1e6)/ (len(p)-1)
            s.num_joints = len(p[i].positions)
            s.joint_name = joint_trajectory.joint_names
            s.joint_position = p[i].positions
            s.joint_velocity = p[i].velocities
            s.joint_effort = p[i].effort

            #print '----- Trajectory Point #', i+1, '-------'
            #print 't:', s.utime, '\t Num joints:', s.num_joints
            #print s.joint_position
            #print s.joint_velocity
            #print s.joint_effort

            plan.plan.append(s)
            self.command_positions = s.joint_position

    	print 'published plan'
    	self.lc.publish('COMMITTED_ROBOT_PLAN', plan.encode())
        self.robot_conf_state = 'transcient'

        #rospy.sleep(5.0)
        #print 'Converging to error', max(self.joint_positions-last_position)
        #if max(self.joint_positions-last_position) < 0.01:
        #    self.lc.publish("CANCEL_PLAN",lcmt_iiwa_status().encode() )
        #    print 'Finished and canceling plan'
        return duration

    def iiwa_status_sub(self, channel, data):
        self.joint_positions = lcmt_iiwa_status.decode(data).joint_position_measured
        #print 'iiwa status', self.joint_positions


    def handle_send_trajectory(self, req):
        resp = SendJointTrajectoryResponse()
        duration = self.trajectory_runner(req.trajectory, req.duration)
        resp.success = wait_for_convergence(self, timeout=duration)
        return resp

    def runOnce(self):
        self.lc.handle()
        self.robot_conf_state = update_conf_state(self.robot_conf_state, self.joint_positions, self.command_positions, isDict=False)

    def runServer(self):
        s1 = rospy.Service('/robot_control/SendJointTrajectory', SendJointTrajectory, self.handle_send_trajectory)


if __name__ == '__main__':
    rospy.init_node('TrajectoryServer')
    fs = TrajectoryServer()
    print "Starting TrajectoryServer"
    fs.runServer()
    while not rospy.is_shutdown():
        fs.runOnce()
        fs.rate.sleep()
