from director import consoleapp
from director import lcmUtils
from director import robotstate
from director import drcargs
from director.utime import getUtime
import drake as lcmdrake
import bot_core as lcmbotcore
import numpy as np
import argparse


class KukaIiwaStateTranslator(object):

    def __init__(self):
        self.lastGripperMsg = self.makeDefaultGripperMessage()
        self.onSchunkGripperStatus(self.lastGripperMsg)
        self.setupJointNames()

    def makeDefaultGripperMessage(self):
        lastGripperMsg = lcmdrake.lcmt_schunk_wsg_status()
        lastGripperMsg.actual_position_mm = 0.0
        return lastGripperMsg

    def setupJointNames(self):
        self.fingerJointNames = ['wsg_50_finger_left_joint', 'wsg_50_finger_right_joint']

        self.armJointNames = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']

        self.jointNames = self.armJointNames + self.fingerJointNames

        self.numJoints = len(self.jointNames)

    def onSchunkGripperStatus(self, msg):
        self.lastGripperMsg = msg
        self.fingerJointPositions = self.getFingerJointPositions(msg)

    def getFingerJointPositions(self, gripperMsg):
        fingerHalfDist = gripperMsg.actual_position_mm * 1e-3 * 0.5
        fingerJointPositions = [fingerHalfDist, fingerHalfDist]
        return fingerJointPositions

    def onIiwaStatus(self, msg):

        armJointPositions = list(msg.joint_position_measured)

        jointPosition = armJointPositions + self.fingerJointPositions

        m = lcmbotcore.robot_state_t()
        # m.utime = msg.utime                                        # this used to get utimes from the kuka robot.  should later fix in drake-iiwa-driver/src/kuka_driver.cc
        m.utime = getUtime()
        m.pose = robotstate.getPoseLCMFromXYZRPY([0,0,0], [0,0,0])
        m.twist = lcmbotcore.twist_t()
        m.twist.linear_velocity = lcmbotcore.vector_3d_t()
        m.twist.angular_velocity = lcmbotcore.vector_3d_t()
        m.num_joints = self.numJoints
        m.joint_name = self.jointNames
        m.joint_position = jointPosition
        m.joint_velocity = np.zeros(m.num_joints)
        m.joint_effort = np.zeros(m.num_joints)
        m.force_torque = lcmbotcore.force_torque_t()
        m.force_torque.l_hand_force = np.zeros(3)
        m.force_torque.l_hand_torque = np.zeros(3)
        m.force_torque.r_hand_force = np.zeros(3)
        m.force_torque.r_hand_torque = np.zeros(3)

        lcmUtils.publish('EST_ROBOT_STATE', m)

    def onIiwaStateEst(self, msg):
        armJointPositions = list(msg.joint_position)
        jointPosition = armJointPositions + self.fingerJointPositions

        # update the num_joints, joint_name, joint_position 
        # joint_velocity, joint_effot fields
        msg.num_joints = self.numJoints
        msg.joint_name = self.jointNames
        msg.joint_position = jointPosition
        msg.joint_velocity = np.zeros(msg.num_joints)
        msg.joint_effort = np.zeros(msg.num_joints)

        lcmUtils.publish('EST_ROBOT_STATE', msg)



if __name__ == "__main__":

    parser = drcargs.getGlobalArgParser().getParser()
    parser.add_argument('--useIiwaStateEst', action='store_true', help='republishes IIWA_STATE_EST on EST_ROBOT_STATE channel instead translating from IIWA_STATUS_MESSAGE')
    args = parser.parse_args()

    kukaStateTranslator = KukaIiwaStateTranslator()

    if args.useIiwaStateEst:
        # for use when running with drake simulator
        print "EST_ROBOT_STATE coming from IIWA_STATE_EST"
        subscriber = lcmUtils.addSubscriber(
    'IIWA_STATE_EST', lcmbotcore.robot_state_t, kukaStateTranslator.onIiwaStateEst)
    else:
        # for use when running with iiwa hardware
        print "EST_ROBOT_STATE coming from IIWA_STATUS"
        subscriber = lcmUtils.addSubscriber(
    'IIWA_STATUS', lcmdrake.lcmt_iiwa_status, kukaStateTranslator.onIiwaStatus)


    subscriber = lcmUtils.addSubscriber(
    'SCHUNK_WSG_STATUS', lcmdrake.lcmt_schunk_wsg_status, kukaStateTranslator.onSchunkGripperStatus)
    consoleapp.ConsoleApp.start()


