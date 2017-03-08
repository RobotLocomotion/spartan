from director import consoleapp
from director import lcmUtils
from director import robotstate
import drake as lcmdrake
import bot_core as lcmbotcore
import numpy as np


lastGripperMsg = lcmdrake.lcmt_schunk_wsg_status()
lastGripperMsg.actual_position_mm = 0.0


def onSchunkGripperStatus(msg):
    global lastGripperMsg
    lastGripperMsg = msg


def onIiwaStatus(msg):

    fingerHalfDist = lastGripperMsg.actual_position_mm * 1e-3 * 0.5
    fingerJointNames = ['wsg_50_finger_left_joint', 'wsg_50_finger_right_joint']
    fingerJointPositions = [fingerHalfDist, fingerHalfDist]

    armJointNames = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
    armJointPositions = list(msg.joint_position_measured)

    jointNames = armJointNames + fingerJointNames
    jointPositions = armJointPositions + fingerJointPositions

    m = lcmbotcore.robot_state_t()
    m.utime = msg.utime
    m.pose = robotstate.getPoseLCMFromXYZRPY([0,0,0], [0,0,0])
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


    lcmUtils.publish('EST_ROBOT_STATE', m)


subscriber = lcmUtils.addSubscriber(
    'IIWA_STATUS', lcmdrake.lcmt_iiwa_status, onIiwaStatus)
subscriber = lcmUtils.addSubscriber(
    'SCHUNK_WSG_STATUS', lcmdrake.lcmt_schunk_wsg_status, onSchunkGripperStatus)

consoleapp.ConsoleApp.start()
