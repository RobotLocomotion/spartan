# ROS
import rospy
import sensor_msgs.msg
import wsg_50_common.msg


class SchunkROSStateTranslator(object):

    def __init__(self):
        self.setupJointNames()
        self._default_joint_state_msg = self.makeDefaultJointStateMessage()
        self.setupPublishers()
        self.gripper_subscriber = rospy.Subscriber("/wsg50_driver/wsg50/status",
            wsg_50_common.msg.Status, self.onSchunkStatus, queue_size=1)

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
        joint_state_msg.name = self.fingerJointNames
        joint_state_msg.position = [0] * self.numJoints
        joint_state_msg.velocity = [0] * self.numJoints
        joint_state_msg.effort = [0] * self.numJoints

        return joint_state_msg

    def setupJointNames(self):
        self.fingerJointNames = ['wsg_50_base_joint_gripper_left', 'wsg_50_base_joint_gripper_right']

        self.jointNames = self.fingerJointNames
        self.numJoints = len(self.jointNames)

    def onSchunkStatus(self, msg):
        joint_state_msg = self._default_joint_state_msg
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.position = [-msg.width*0.5, msg.width*0.5]
        joint_state_msg.velocity = [-msg.current_speed*0.5, msg.current_speed*0.5]
        joint_state_msg.effort = [-msg.current_force, msg.current_force]

        self._joint_states_publisher.publish(joint_state_msg)


if __name__ == "__main__":

    rospy.init_node("schunk_state_translator")

    schunkStateTranslator = SchunkROSStateTranslator()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
