#!/usr/bin/env python

# system
import numpy as np, math


# drake + LCM
from drake import lcmt_iiwa_command, lcmt_iiwa_status ,lcmt_robot_state
from bot_core import robot_state_t
import lcm


# ROS standard library
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


# ROS custom packages
from robot_msgs.srv import *
from robot_msgs.msg import *

class JointStatePublisher:
    def __init__(self):
        self.rate = rospy.Rate(125)

        self.robot_state = JointState()
        self.pub_joint_state = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.joint_names = []
        self.joint_positions = []
        self.joint_velocities = []
        self.joint_efforts = []
        self.joint_idx = {}

        self.lc = lcm.LCM()
        self.lc.subscribe("IIWA_STATUS", self.onIiwaStatus)
        #gripper_sub = self.lc.subscribe("command_topic", self.gripper_sub)
        
        # self.base_names = ['base_x', 'base_y', 'base_theta']
        self.iiwa_joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
        #self.gripper_names = ['robotiq_hand'] # !! Jay, maybe want the "actual" name...not that it matters...

        self.joints = []
        # self.joints.extend(self.base_names)
        self.joints.extend(self.iiwa_joint_names)
        #self.joints.extend(self.gripper_names)

        idx = 0
        for j in xrange(0, len(self.joints)):
            self.joint_names.append(str(self.joints[j]))
            self.joint_positions.append(0.0)
            self.joint_velocities.append(0.0)
            self.joint_efforts.append(0.0)
            self.joint_idx[str(self.joints[j])] = idx
            idx = idx + 1

    def onIiwaStatus(self, channel, data):
        msg = lcmt_iiwa_status.decode(data)
        for data_idx in xrange(0, len(self.iiwa_joint_names)):
            joint_name = self.iiwa_joint_names[data_idx]
            if joint_name in self.joint_idx:
                idx = self.joint_idx[joint_name]
                self.joint_positions[idx] = msg.joint_position_measured[data_idx]
                self.joint_velocities[idx] = msg.joint_velocity_estimated[data_idx] # TODO(gizatt) See which other fields are valid and add them here
                self.joint_efforts[idx] = msg.joint_torque_measured[data_idx]

    def run(self):
        self.lc.handle_timeout(0.005)
        self.publishROSJointStateMessage()

    def publishROSJointStateMessage(self):
        self.robot_state.header = Header()
        self.robot_state.header.stamp = rospy.Time.now()
        self.robot_state.name = self.joint_names
        self.robot_state.position = self.joint_positions
        self.robot_state.velocity = self.joint_velocities
        self.robot_state.effort = self.joint_efforts
        self.pub_joint_state.publish(self.robot_state)

    def runServer(self):
        #s1 = rospy.Service('robot_server/getRobotConf', GetRobotConf, self.handle_get_conf)
        pass

if __name__ == '__main__':
    rospy.init_node('JointStatePublisher')
    jp = JointStatePublisher()
    print "Starting JointStatePublisher"
    jp.runServer()
    while not rospy.is_shutdown():
        jp.run()
        jp.rate.sleep()
