from director import consoleapp
from director import lcmUtils
from director import robotstate
from director import drcargs
from director.utime import getUtime
import drake as lcmdrake
import bot_core as lcmbotcore
import numpy as np
import argparse

# ROS
import rospy
import sensor_msgs.msg


class KukaIiwaROSStateTranslator(object):

    def __init__(self, basePosition):
        self.basePosition = basePosition
        self.setupJointNames()
        self._default_joint_state_msg = self.makeDefaultJointStateMessage()
        self.setupPublishers()

    def setupPublishers(self):
        """
        Initializes the ROS publishers
        """
        
        self._joint_states_publisher = rospy.Publisher("/joint_states", sensor_msgs.msg.JointState, queue_size=1)

    def makeDefaultJointStateMessage(self):
        """
        Makes a sensor_msgs/JointState msg with appropriate joint_names
        All other fields are intialized to zero
        """

        joint_state_msg = sensor_msgs.msg.JointState()
        joint_state_msg.name = self.armJointNames
        joint_state_msg.position = [0] * self.numJoints
        joint_state_msg.velocity = [0] * self.numJoints
        joint_state_msg.effort = [0] * self.numJoints

        return joint_state_msg

    def setupJointNames(self):
        self.armJointNames = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']

        # self.jointNames = self.armJointNames + self.fingerJointNames
        self.jointNames = self.armJointNames
        self.numJoints = len(self.jointNames)

    def onIiwaStatus(self, msg):
        """
        :param msg: the IIWA_STATUS message
        :type msg: lcmt.iiwa_status_t

        Translates the IIWA_STATUS message to sensor_msgs/JointState format
        and publishes
        """
        joint_state_msg = self._default_joint_state_msg
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.position = list(msg.joint_position_measured)
        joint_state_msg.velocity = list(msg.joint_velocity_estimated)
        joint_state_msg.effort = list(msg.joint_torque_external)
        
        self._joint_states_publisher.publish(joint_state_msg)


if __name__ == "__main__":

    rospy.init_node("iiwa_state_translator")

    parser = drcargs.getGlobalArgParser().getParser()
    
    parser.add_argument("--basePosition", help="Base position of Kuka, formatted \"0.0, 0.0, 0.0, 0.0, 0.0, 0.0\"", type=str, default="0., 0., 0., 0., 0., 0.")
    args = parser.parse_args()

    basePosition = np.array([float(f) for f in args.basePosition.split(",")])
    print "Using base position ", basePosition
    kukaStateTranslator = KukaIiwaROSStateTranslator(basePosition)

    subscriber = lcmUtils.addSubscriber(
    'IIWA_STATUS', lcmdrake.lcmt_iiwa_status, kukaStateTranslator.onIiwaStatus)

    consoleapp.ConsoleApp.start()


