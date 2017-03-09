from director import consoleapp
from director import lcmUtils
from director import robotstate

import robotlocomotion as robotlocomotion_lcmtypes
import drake as lcmdrake

def onIiwaStatus(msg):
    joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
    num_joints = len(joint_names)

    # construct residual message
    residual_msg = robotlocomotion_lcmtypes.residual_observer_state_t()
    residual_msg.utime = msg.utime
    residual_msg.num_joints = num_joints
    residual_msg.joint_name = joint_names

    # initialize fields
    residual_msg.residual = [0]*num_joints
    residual_msg.gravity = [0]*num_joints
    residual_msg.internal_torque = [0]*num_joints
    residual_msg.foot_contact_torque = [0]*num_joints

    # fill in residual field
    for i in xrange(0,residual_msg.num_joints):
    	residual_msg.residual[i] = msg.joint_torque_external[i]

    lcmUtils.publish('RESIDUAL_OBSERVER_STATE', residual_msg)


subscriber = lcmUtils.addSubscriber(
    'IIWA_STATUS', lcmdrake.lcmt_iiwa_status, onIiwaStatus)
consoleapp.ConsoleApp.start()
