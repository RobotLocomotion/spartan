#!/usr/bin/env python
import rospy
import numpy as np
import actionlib
import robotlocomotion
import bot_core as lcmbotcore
from robotlocomotion import robot_plan_t
from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *
from tf2_msgs.msg import *
from sensor_msgs.msg import JointState
import lcm
import time

client = None
lc = lcm.LCM()

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def handlePlan(channel, data):
    print("got plan message")
    msg = robot_plan_t.decode(data)
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = []
    joint_states = rospy.wait_for_message("joint_states", JointState)
    joints_pos = joint_states.position
    g.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)))
    print("debug")
    print(msg.plan[0].utime)
    print(msg.plan[1].utime)
    #print(msg.plan)
    for state in msg.plan:
        g.trajectory.points.append(JointTrajectoryPoint(positions=state.joint_position, velocities=state.joint_velocity, time_from_start=rospy.Duration(state.utime*1e-6)))
    client.send_goal(g)
    client.wait_for_result()
    print("sent")

def handleState(msg):
    #rospy.loginfo("hello")
    armJointNames = JOINT_NAMES
    armJointPositions = list(msg.position)

    jointNames = armJointNames
    jointPositions = armJointPositions

    m = lcmbotcore.robot_state_t()
    m.utime = time.clock() * 1e6
    #m.pose = robotstate.getPoseLCMFromXYZRPY([0,0,0], [0,0,0])
    m.twist = lcmbotcore.twist_t()
    m.twist.linear_velocity = lcmbotcore.vector_3d_t()
    m.twist.angular_velocity = lcmbotcore.vector_3d_t()
    m.num_joints = len(jointNames)
    m.joint_name = jointNames
    m.joint_position = jointPositions
    m.joint_velocity = np.zeros(m.num_joints)
    m.joint_effort = np.zeros(m.num_joints)
    m.force_torque = lcmbotcore.force_torque_t()
    m.force_torque.l_hand_force = np.zeros(3)
    m.force_torque.l_hand_torque = np.zeros(3)
    m.force_torque.r_hand_force = np.zeros(3)
    m.force_torque.r_hand_torque = np.zeros(3)
    lc.publish('EST_ROBOT_STATE', m.encode())

rospy.init_node('lcm_interface', anonymous=True)

print("connecting")
client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()
print("connected")

sub = lc.subscribe("COMMITTED_ROBOT_PLAN", handlePlan)
subscriber = rospy.Subscriber("joint_states", JointState, handleState, queue_size=200)
joint_states = rospy.wait_for_message("joint_states", JointState)
print("got msg")
print(joint_states.position)
print("spinning")
#consoleapp.ConsoleApp.start()
try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass
rospy.spin()
