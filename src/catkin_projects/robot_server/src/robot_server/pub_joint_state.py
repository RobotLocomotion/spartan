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
from wsg50_msgs.msg import *

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
        self.gripper_subscriber = rospy.Subscriber("/schunk_driver/schunk_wsg_status", 
            WSG_50_state, self.onSchunkStatus, queue_size=1)

        #gripper_sub = self.lc.subscribe("command_topic", self.gripper_sub)
        
        # self.base_names = ['base_x', 'base_y', 'base_theta']
        self.iiwa_joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
        self.gripper_names = ['wsg_50_base_joint_gripper_left', 'wsg_50_base_joint_gripper_right']

        self.joints = []
        # self.joints.extend(self.base_names)
        self.joints.extend(self.iiwa_joint_names)
        self.joints.extend(self.gripper_names)

        idx = 0
        for j in xrange(0, len(self.joints)):
            self.joint_names.append(str(self.joints[j]))
            self.joint_positions.append(0.0)
            self.joint_velocities.append(0.0)
            self.joint_efforts.append(0.0)
            self.joint_idx[str(self.joints[j])] = idx
            idx = idx + 1

    def onIiwaStatus(self, channel, data):
        ros_time_now =  rospy.Time.now()
        
        msg = lcmt_iiwa_status.decode(data)

        print "##############################"
        ros_time_converted_from_lcm = rospy.Time(0,msg.utime*1000)
        print ros_time_converted_from_lcm, "is a rospy time created from the lcm timestamp"

        # latency debug printing
        lcm_time_stamp_utime = msg.utime
        print lcm_time_stamp_utime, "is utime from lcm"
        print ros_time_now, "is rospy time"
        print ros_time_now.to_sec(), "is rospy time.to_sec()"
        print ros_time_now.to_nsec()/1000, "is rospy time.to_nsec()/1000"
        print type(ros_time_now.to_nsec()), type(lcm_time_stamp_utime)
        ros_time_converted_from_lcm_utime = ros_time_converted_from_lcm.to_nsec()/1000
        print ros_time_converted_from_lcm_utime - lcm_time_stamp_utime, "is offset"
        print 

        for data_idx in xrange(0, len(self.iiwa_joint_names)):
            joint_name = self.iiwa_joint_names[data_idx]
            if joint_name in self.joint_idx:
                idx = self.joint_idx[joint_name]
                self.joint_positions[idx] = msg.joint_position_measured[data_idx]
                self.joint_velocities[idx] = msg.joint_velocity_estimated[data_idx] # TODO(gizatt) See which other fields are valid and add them here
                self.joint_efforts[idx] = msg.joint_torque_measured[data_idx]

        # We *should* be using the sunrise cabinet time.
        # But it's not always well synchronized with our time,
        # (e.g. at time of writing, it was off by hours and was
        # messing up the TF server), so for now I'm approximating with
        # our local system time. -gizatt
        self.publishROSJointStateMessage(ros_time_now)

    def onSchunkStatus(self, msg):
        ros_time_now =  rospy.Time.now()
        

        # Left finger
        idx = self.joint_idx['wsg_50_base_joint_gripper_left']
        self.joint_positions[idx] = -msg.position_mm * 0.0005
        self.joint_velocities[idx] = -msg.speed_mm_per_s * 0.0005
        self.joint_efforts[idx] = -msg.force

        # Right finge
        idx = self.joint_idx['wsg_50_base_joint_gripper_right']
        self.joint_positions[idx] = msg.position_mm * 0.0005
        self.joint_velocities[idx] = msg.speed_mm_per_s * 0.0005
        self.joint_efforts[idx] = msg.force
        
        # This could be enabled, but I'm going to let Iiwa
        # statuses be the driver of when status gets
        # published out, to avoid unnecessary traffic.
        # self.publishROSJointStateMessage(ros_time_now)

    def run(self):
        self.lc.handle_timeout(0.005)
        #print "LOOPING INSIDE pub_joint_state"
        #print ""

    def publishROSJointStateMessage(self, ros_time_now_previously_grabbed):
        self.robot_state.header = Header()
        self.robot_state.header.stamp = ros_time_now_previously_grabbed
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
