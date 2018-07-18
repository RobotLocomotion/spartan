import rospy, time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_msgs.msg import *
from robot_msgs.srv import *
import rospy

def getIiwaJointNames():
    return ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',
     'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6',
     'iiwa_joint_7']
