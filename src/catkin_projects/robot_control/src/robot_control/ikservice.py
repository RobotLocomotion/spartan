import os
import numpy as np
import yaml

from director import visualization as vis
from director import objectmodel as om
from director import transformUtils
from director import ioUtils
from director import filterUtils
from director import vtkAll as vtk
from director.debugVis import DebugData
from director import segmentation
from director import lcmUtils
from director import lcmframe
from director import robotstate
from director.ikplanner import ConstraintSet
from director.timercallback import TimerCallback
from director.ikparameters import IkParameters


# spartan
from spartan.utils.taskrunner import TaskRunner

#ROS
import rospy
import sensor_msgs.msg

# ROS custom packages
import robot_msgs.srv
import robot_control.control_utils as controlUtils




class IkService(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.endEffectorLinkName = 'iiwa_link_ee'
        self.jointNames = controlUtils.getIiwaJointNames()
        self.numJoints = len(self.jointNames)

        self.config = dict()
        self.config['ikservice_name'] = "robot_control/IkService"

    def runIK(self, targetFrame, startPose=None, graspToHandLinkFrame=None, positionTolerance=0.0, angleToleranceInDegrees=0.0, seedPoseName='q_nom'):

        if graspToHandLinkFrame is None:
            graspToHandLinkFrame = vtk.vtkTransform()

        if startPose is None:
            startPose = self.getPlanningStartPose()

        ikPlanner = self.robotSystem.ikPlanner
        startPoseName = 'reach_start'
        endPoseName = 'reach_end'
        ikPlanner.addPose(startPose, startPoseName)
        side = ikPlanner.reachingSide

        constraints = []
        constraints.append(KukaPlanningUtils.createLockedBasePostureConstraint(ikPlanner, startPoseName))

        # positionConstraint, orientationConstraint = ikPlanner.createPositionOrientationGraspConstraints(side, targetFrame,
        #                                                                                            graspToHandLinkFrame,
        #                                                                                            positionTolerance=positionTolerance,
        #                                                                                            angleToleranceInDegrees=angleToleranceInDegrees)

        positionConstraint, orientationConstraint = ikPlanner.createPositionOrientationConstraint(self.endEffectorLinkName,
                                                                                                        targetFrame,
                                                                                                        graspToHandLinkFrame,
                                                                                                        positionTolerance=positionTolerance,
                                                                                                        angleToleranceInDegrees=angleToleranceInDegrees)

        positionConstraint.tspan = [1.0, 1.0]
        orientationConstraint.tspan = [1.0, 1.0]

        constraints.append(positionConstraint)
        constraints.append(orientationConstraint)

        constraintSet = ConstraintSet(ikPlanner, constraints, 'reach_end',
                                      startPoseName)
        constraintSet.ikParameters = IkParameters()

        constraintSet.seedPoseName = seedPoseName

        print "consraintSet ", constraintSet
        print "constraintSet.seedPoseName ", constraintSet.seedPoseName
        print "constraintSet.nominalPoseName ", constraintSet.nominalPoseName


        endPose, info = constraintSet.runIk()
        returnData = dict()
        returnData['info'] = info
        returnData['endPose'] = endPose



        return returnData

    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q

    def drakeJointPositionToRosJointState(self, q):
        # remove the floating base . . .
        q = q[-self.numJoints:]

        jointState = sensor_msgs.msg.JointState()
        jointState.header.stamp = rospy.Time.now()
        jointState.name = self.jointNames
        jointState.position = q
        jointState.velocity = [0]*self.numJoints
        jointState.effort = [0]*self.numJoints

        return jointState


    def onIkServiceRequest(self, req):
        rospy.loginfo("received an IkService request")
        targetFrame = IkService.ROSPoseToVtkTransform(req.pose_stamped.pose)

        ikResult = self.runIK(targetFrame)

        rospy.loginfo("IK info = %d", ikResult['info'])

        response = robot_msgs.srv.RunIKResponse()
        response.success = (ikResult['info'] == 1)
        rospy.loginfo("IK solution found = %s", response.success)
        response.joint_state = self.drakeJointPositionToRosJointState(ikResult['endPose'])

        return response

    def advertiseServices(self):
        rospy.loginfo("advertising services")
        self.ikService = rospy.Service(self.config['ikservice_name'],
                                                        robot_msgs.srv.RunIK, self.onIkServiceRequest)


    # run this in a thread
    def runRosNode(self):
        self.advertiseServices()
        rospy.loginfo("IkService ready!")
        while not rospy.is_shutdown():
            rospy.spin()


    def run(self):
        rospy.init_node("IkService")
        self.taskRunner = TaskRunner()
        self.taskRunner.callOnThread(self.runRosNode)


    def test(self):
        pos = [2.00213459e-01, -1.93298822e-12, 8.99913227e-01]
        quat = [9.99999938e-01, -1.16816462e-12, 3.51808464e-04, -5.36917849e-12]
        targetFrame = transformUtils.transformFromPose(pos,quat)
        return self.runIK(targetFrame)

    """
    @:param pose: geometry_msgs/Pose
    """
    @staticmethod
    def ROSPoseToVtkTransform(pose):
        pos = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        return transformUtils.transformFromPose(pos, quat)


class KukaPlanningUtils(object):

    @staticmethod
    def createLockedBasePostureConstraint(ikPlanner, startPoseName):
        return ikPlanner.createPostureConstraint(startPoseName, robotstate.matchJoints('base_'))

