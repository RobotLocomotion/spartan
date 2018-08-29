#!/usr/bin/env python

# system
import numpy as np

# ROS
import rospy
import sensor_msgs.msg
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ROS custom packages
import robot_msgs.srv
import robot_msgs.msg
import robot_control.control_utils as controlUtils


from spartan.utils.ros_utils import JointStateSubscriber


class RobotMovementService(object):
    """
    Helper class that provides some basic functionality, i.e. moveToJointPosition
    """

    def __init__(self, config):

        self.config = config
        self.setupActionClients()
        self.setupRobot()
        self.setupSubscribers()

    def setupActionClients(self):
        """
        Connects to the action server for the plan_runner
        :return: None
        :rtype:
        """

        rospy.loginfo("waiting for the action %s", self.config["joint_space_trajectory_action"])
        self.joint_space_trajectory_action = actionlib.SimpleActionClient(self.config["joint_space_trajectory_action"], robot_msgs.msg.JointTrajectoryAction)

        self.joint_space_trajectory_action.wait_for_server()

    def setupSubscribers(self):
        self.subscribers = dict()


        s = JointStateSubscriber(topic=self.config['joint_states_topic'])
        self.subscribers['joint_states'] = s

    def setupRobot(self):
        self.jointNames = controlUtils.getIiwaJointNames()

    def getRobotState(self):
        self.subscribers['joint_states'].subscriber.waitForNextMessage()
        reduced_state = sensor_msgs.msg.JointState()
        reduced_state.name = self.jointNames
        reduced_state.position = self.subscribers['joint_states']\
            .get_position_vector_from_joint_names(self.jointNames)
        reduced_state.velocity = self.subscribers['joint_states']\
            .get_velocity_vector_from_joint_names(self.jointNames)
        reduced_state.effort = self.subscribers['joint_states']\
            .get_effort_vector_from_joint_names(self.jointNames)
        return reduced_state

    """
    @:param req: MoveToJointPosition.srv
    """
    def moveToJointPosition(self, req):
        """
        Callback for the MoveToJointPosition service

        Constructs a joint space trajectory plan that interpolates from the current position to
        the desired position in joint space
        :param req: robot_msgs.srv.MoveToJointPositionRequest
        :type req:
        :return:
        :rtype:
        """
    
        jointStateFinal = req.joint_state
        jointStateStart = self.getRobotState()
    
        rospy.loginfo("moving robot to joint position %s", str(jointStateFinal.position))

        # figure out the duration based on max joint degrees per second
        print "jointStateStart \n", jointStateStart
        print "jointStateFinal \n", jointStateFinal
        print "jointStateStart.position = ", jointStateStart.position
        q_start = np.array(jointStateStart.position)
        q_end = np.array(jointStateFinal.position)

        minPlanTime = 1.0
        maxDeltaRadians = np.max(np.abs(q_end - q_start))
        duration = maxDeltaRadians/np.deg2rad(req.max_joint_degrees_per_second)
        duration = max(duration, minPlanTime)

        rospy.loginfo("rescaled plan duration is %.2f seconds", duration)


        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()

        startTime = rospy.Duration.from_sec(0)
        endTime = rospy.Duration.from_sec(duration)

        startPoint = RobotMovementService.jointTrajectoryPointFromJointState(jointStateStart, startTime)
        endPoint = RobotMovementService.jointTrajectoryPointFromJointState(jointStateFinal, endTime)


        trajectory.points = [startPoint, endPoint]
        trajectory.joint_names = self.jointNames

        joint_traj_action_goal = robot_msgs.msg.JointTrajectoryGoal()
        joint_traj_action_goal.trajectory = trajectory


        self.joint_space_trajectory_action.send_goal(joint_traj_action_goal)
        self.joint_space_trajectory_action.wait_for_result()
        result = self.joint_space_trajectory_action.get_result()

        finished_normally = (result.status.status == result.status.FINISHED_NORMALLY)
        return finished_normally

    # send this out over the TrajectoryService, wait for response then respond
    def advertiseServices(self):
        rospy.loginfo("advertising services")
        self.moveToJointPositionService = rospy.Service(self.config['move_to_joint_position_service_name'], robot_msgs.srv.MoveToJointPosition, self.moveToJointPosition)

    """
    timeFromStart is a rospy.Duration object
    """
    @staticmethod
    def jointTrajectoryPointFromJointState(jointState, timeFromStart):
        trajPoint = JointTrajectoryPoint()
        trajPoint.positions = jointState.position

        numJoints = len(jointState.position)
        trajPoint.velocities = [0]*numJoints
        trajPoint.accelerations = [0]*numJoints
        trajPoint.effort = [0]*numJoints

        trajPoint.time_from_start = timeFromStart
        return trajPoint

# if __name__ == "__main__":
#     rospy.init_node("RobotMovementService")
#     robotMovementService = RobotMovementService()
#     robotMovementService.advertiseServices()
#     rospy.loginfo("RobotMovementService ready!")
#     while not rospy.is_shutdown():
#         rospy.spin()

