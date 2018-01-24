#!/usr/bin/env python

# system
import numpy as np

# ROS
import rospy
import sensor_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ROS custom packages
import robot_msgs.srv
import robot_control.control_utils as controlUtils


from spartan.utils.ros_utils import SimpleSubscriber


class RobotMovementService(object):

    def __init__(self, joint_states_topic="/joint_states"):

        self.config = dict()
        self.config['joint_states_topic'] = joint_states_topic
        self.config['trajectory_service_name'] = 'robot_control/SendJointTrajectory'
        self.config['move_to_joint_position_service_name'] = "robot_control/MoveToJointPosition"

        self.setupServiceProxies()
        self.setupRobot()
        self.setupSubscribers()

    def setupServiceProxies(self):
        rospy.loginfo("waiting for the service %s", self.config['trajectory_service_name'])
        rospy.wait_for_service(self.config['trajectory_service_name'])
        self.trajectoryService = rospy.ServiceProxy(self.config['trajectory_service_name'],
                                                    robot_msgs.srv.SendJointTrajectory)

    def setupSubscribers(self):
        self.subscribers = dict()


        s = SimpleSubscriber(self.config['joint_states_topic'], sensor_msgs.msg.JointState)
        s.start()
        self.subscribers['joint_states'] = s

    def setupRobot(self):
        self.jointNames = controlUtils.getIiwaJointNames()

    def getRobotState(self):
        full_state = self.subscribers['joint_states'].waitForNextMessage()
        # Reduce to just this robot's state
        keep_indices = [full_state.name.index(name) for name in self.jointNames if name in full_state.name]
        reduced_state = sensor_msgs.msg.JointState()
        reduced_state.name = self.jointNames
        reduced_state.position = [full_state.position[i] for i in keep_indices]
        reduced_state.velocity = [full_state.velocity[i] for i in keep_indices]
        reduced_state.effort = [full_state.effort[i] for i in keep_indices]
        return reduced_state

    """
    @:param req: MoveToJointPosition.srv
    """
    def moveToJointPosition(self, req):
    
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


        trajectoryServiceResponse = self.trajectoryService(trajectory)
        resp = robot_msgs.srv.MoveToJointPositionResponse(trajectoryServiceResponse.success)
        return resp



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

if __name__ == "__main__":
    rospy.init_node("RobotMovementService")
    robotMovementService = RobotMovementService()
    robotMovementService.advertiseServices()
    rospy.loginfo("RobotMovementService ready!")
    while not rospy.is_shutdown():
        rospy.spin()

