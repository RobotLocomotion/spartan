#system
import numpy as np
import time
import os
import subprocess



#director
from director import lcmUtils
from director import lcmframe
from director import transformUtils
from director import robotsystem
from director import segmentation
from director import cameraview
from director import pydrakeik
from director import packagepath
from director import roboturdf
from director import robotlinkselector
from director import fieldcontainer
from director import framevisualization
from director import drcargs
from director import visualization as vis
from director import ikplanner
from director import ikconstraints
from director import robotposegui
from director.timercallback import TimerCallback
from director import robotstate
from director.ikplanner import ConstraintSet
import director.vtkAll as vtk



RobotPoseGUIWrapper = ikplanner.RobotPoseGUIWrapper
import bot_core as lcmbotcore
from director.ikparameters import IkParameters


try:
    from labelfusion.cameraposes import CameraPoses
except:
    print "Have you sourced setup_environment_labelfusion.sh in spartan/config?"
    quit()

# spartan
import spartan.utils as spartanUtils
from spartan.taskrunner import TaskRunner


# ROS
import rospy
import sensor_msgs.msg

# ROS custom
import robot_msgs.srv

"""
To run this set useKukaRLGDev to True in iiwaManipApp.py. This loads a
HandEyeCalibration object into the director workspace, it is called cal.

To run calibration simply press F8 and enter cal.runThreaded(). This creates a new directory
in spartan/calibration_data which contains two files.

calibration.lcmlog: a log of the run
robot_data.yaml: relevant data for the "hand_link" poses during the run.

To finish the process go to the log folder mentioned above and run

run_elastic_fusion_on_log.py -l calibration.lcmlog
add_camera_poses_to_calibration_data.py

Then the file camera_poses_and_robot_data.yaml contains all the information
needed to run an AX=XB style hand-eye calibration.



"""

class RobotService(object):

    def __init__(self, robotSystem, removeFloatingBase=True):
        self.robotSystem = robotSystem

        self.jointNames = self.robotSystem.ikPlanner.robotModel.model.getJointNames()
        if removeFloatingBase:
            self.jointNames = self.jointNames[6:]
            self.numJoints = len(self.jointNames)

        self.manipPlanner = robotSystem.manipPlanner
        self.removeFloatingBase = True
        self.ikPlanner = robotSystem.ikPlanner

    """
    Joint positions should be a dict of the form{'joint_name': joint_value}

    """

    def moveToJointPosition(self, q):
        if self.removeFloatingBase:
            q = q[-self.numJoints:]

        maxJointDegreesPerSecond = 30

        jointState = sensor_msgs.msg.JointState()
        jointState.header.stamp = rospy.Time.now()

        jointState.position = q
        jointState.name = self.jointNames

        jointState.velocity = [0] * self.numJoints
        jointState.effort = [0] * self.numJoints

        rospy.wait_for_service('robot_control/MoveToJointPosition')
        s = rospy.ServiceProxy('robot_control/MoveToJointPosition', robot_msgs.srv.MoveToJointPosition)
        response = s(jointState, maxJointDegreesPerSecond)



    def movePose(self, joint_positions, maxDegreesPerSecond=30):

        assert isinstance(joint_positions, dict)

        self.manipPlanner.lastPlan = None
        startPose = self.robotSystem.robotStateJointController.q
        endPose = self.ikPlanner.mergePostures(startPose, joint_positions)
        ikParameters = IkParameters(maxDegreesPerSecond=maxDegreesPerSecond)
        plan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=ikParameters)
        self.manipPlanner.commitManipPlan(plan)

        # block until this plan is over
        planDuration = plan.plan[-1].utime/1e6
        print "running plan of duration %s seconds "  %(planDuration)
        time.sleep(1.1*planDuration)

    def getPose(self):
        q = self.robotSystem.robotStateJointController.q
        if self.removeFloatingBase:
            q = q[6:]

        return q


class HandEyeCalibration(object):

    def __init__(self, robotSystem, handFrame='palm'):
        self.robotSystem = robotSystem
        self.robotService = RobotService(robotSystem)
        self.handFrame = handFrame
        self.setup()
        self.calibrationData = None

        self.timer = TimerCallback(targetFps=1)
        self.timer.callback = self.callback
        self.taskRunner = TaskRunner()
    # self.timer.callback = self.callback

    def setup(self):
        self.nominalPose = 'center'
        self.poseList = ['center', 'left', 'right', 'forwards', 'backwards']
        self.groupName = 'Calibration'
        poseNamesInCalibrationGroup = RobotPoseGUIWrapper.getPoseNamesInGroup(self.groupName)
        print "names in calibration group ", poseNamesInCalibrationGroup
        self.poseDict = dict()

        for poseName in self.poseList:
            d = dict()
            d['nominal'] = RobotPoseGUIWrapper.getPose(self.groupName, poseName)

            for extension in ['x', 'y', 'z']:
                poseNameExtended = poseName + "_" + extension
                if poseNameExtended in poseNamesInCalibrationGroup:
                    d[extension] = RobotPoseGUIWrapper.getPose(self.groupName, poseNameExtended)

            self.poseDict[poseName] = d




    def captureDataAtPose(self, poseName=""):
        data = dict()
        data['pose_name'] = poseName
        data['pose'] = self.robotService.getPose().tolist()
        data['hand_frame_name'] = self.handFrame
        handTransform = self.robotSystem.robotStateModel.getLinkFrame(self.handFrame)
        data['hand_frame'] = spartanUtils.poseFromTransform(handTransform)
        data['utime'] = self.robotSystem.robotStateJointController.lastRobotStateMessage.utime
        return data

    def testCaptureData(self):
        data = self.captureDataAtPose()
        self.calibrationData.append(data)

    def moveHome(self):
        self.robotService.movePose(self.poseDict['center']['nominal'])

    def runThreaded(self):
        self.taskRunner.callOnThread(self.run)

    def run(self):
        self.calibrationData = []
        self.moveHome()

        for poseName in self.poseList:

            poses = self.poseDict[poseName]
            nominal = poses['nominal']
            self.robotService.movePose(nominal)
            data = self.captureDataAtPose('nominal')
            self.calibrationData.append(data)

            for subPoseName, pose in poses.iteritems():
                if subPoseName == 'nominal':
                    continue;

                self.robotService.movePose(pose)
                data = self.captureDataAtPose(poseName + "_" + subPoseName)
                self.calibrationData.append(data)
                self.robotService.movePose(nominal)

            self.moveHome()


    def makePoseOrder(self):
        self.poseOrder = []
        for poseName in self.poseList:
            poses = self.poseDict[poseName]
            self.poseOrder.append(self.poseDict[poseName]['nominal'])
            for subPoseName, pose in poses.iteritems():
                if subPoseName == 'nominal':
                    continue;
                self.poseOrder.append(self.poseDict[poseName][subPoseName])

            self.poseOrder.append(self.poseDict['center']['nominal'])

    def callback(self):
        self.testCaptureData()
        if len(self.poseOrder) == 0:
            print "finished data collection"
            self.timer.stop()
            self.loggerProcess.kill()
            self.saveCalibrationData(os.path.join(self.calibrationFolderName, 'robot_data.yaml'))
            return


        pose = self.poseOrder.pop(0)
        self.robotService.movePose(pose)


    def runCalibration(self):
        self.calibrationData = []
        unique_name = time.strftime("%Y%m%d-%H%M%S")
        self.calibrationFolderName = os.path.join(spartanUtils.getSpartanSourceDir(), 'calibration_data',unique_name)
        os.system("mkdir -p " + self.calibrationFolderName)
        os.chdir(self.calibrationFolderName)
        cmd = "lcm-logger calibration.lcmlog"
        self.loggerProcess = subprocess.Popen("exec " + cmd, shell = True)

        # waiting for logger to start
        print "waiting for logger to start . . . "
        time.sleep(2.0) # give the logger process time to start

        self.makePoseOrder()
        self.timer.start()

    def test(self):
        pose = self.poseDict['center']['nominal']
        self.robotService.movePose(pose)

    def saveCalibrationData(self, filename=None):
        if filename is None:
            filename = os.path.join(spartanUtils.getSpartanSourceDir(), 'sandbox', 'hand_eye_calibration_robot_data.yaml')


        spartanUtils.saveToYaml(self.calibrationData, filename)

    def computeCalibrationPoses(self):
        config = dict()
        config['yaw_min'] = -90
        config['yaw_max'] = 90
        config['pitch_min'] = 0
        config['pitch_max'] = 120

        # vector going from the target back towards the robot
        config['target_to_robot'] = [-1,0,0]

    @staticmethod
    def createCameraGazeTargetConstraint(linkName=None, cameraToLinkFrame=None, cameraAxis=None, worldPoint=None, coneThresholdDegrees=0.0):

        print [linkName, cameraToLinkFrame, cameraAxis, worldPoint]
        if None in [linkName, cameraToLinkFrame, cameraAxis, worldPoint]:
            raise ValueError("must specify a field")

        g = ikconstraints.WorldGazeTargetConstraint()
        g.linkName = linkName
        g.tspan = [1.0, 1.0]
        g.worldPoint = worldPoint

        g.bodyPoint = cameraToLinkFrame.GetPosition()

        # axis expressed in link frame
        axis = cameraToLinkFrame.TransformVector(cameraAxis)
        g.axis = axis
        g.coneThreshold = np.deg2rad(coneThresholdDegrees)


        return g

    @staticmethod
    def createPositionConstraint(targetPosition=None, positionTolerance=0.0, linkName=None, linkOffsetFrame=None):
        if linkOffsetFrame is None:
            linkOffsetFrame = vtk.vtkTransform()

        p = ikconstraints.PositionConstraint()
        p.linkName = linkName
        p.pointInLink = np.array(linkOffsetFrame.GetPosition())

        targetFrame = None
        if isinstance(targetPosition, vtk.vtkTransform):
            targetFrame = targetPosition
        else:
            quat = [1,0,0,0]
            targetFrame = transformUtils.transformFromPose(targetPosition, quat)

        p.referenceFrame = targetFrame
        p.lowerBound = np.tile(-positionTolerance, 3)
        p.upperBound = np.tile(positionTolerance, 3)

        return p

    def computeSingleCameraPose(self, targetLocationWorld=[1,0,0], cameraFrameLocation=[0.22, 0, 0.89]):
        cameraAxis = [0,-1,0] # assuming we are using 'palm' as the link frame

        linkName = self.handFrame
        linkName = 'iiwa_link_7'
        linkFrame =self.robotSystem.robotStateModel.getLinkFrame(linkName)
        cameraFrame = self.robotSystem.robotStateModel.getLinkFrame(self.handFrame)

        cameraToLinkFrame = transformUtils.concatenateTransforms([cameraFrame, linkFrame.GetLinearInverse()])
        ikPlanner = self.robotSystem.ikPlanner
        startPoseName = 'q_nom'
        endPoseName = 'reach_end'
        seedPoseName = 'q_nom'

        constraints = []
        constraints.append(ikPlanner.createPostureConstraint(startPoseName, robotstate.matchJoints('base_')))

        positionConstraint = HandEyeCalibration.createPositionConstraint(targetPosition=cameraFrameLocation, linkName=linkName, linkOffsetFrame=cameraToLinkFrame, positionTolerance=0.05)

        cameraGazeConstraint = HandEyeCalibration.createCameraGazeTargetConstraint(linkName=linkName, cameraToLinkFrame=cameraToLinkFrame, cameraAxis=cameraAxis, worldPoint=targetLocationWorld, coneThresholdDegrees=5.0)


        constraints.append(positionConstraint)
        # constraints.append(cameraGazeConstraint)

        constraintSet = ConstraintSet(ikPlanner, constraints, 'reach_end',
                                      startPoseName)
        constraintSet.ikParameters = IkParameters()

        constraintSet.seedPoseName = seedPoseName

        endPose, info = constraintSet.runIk()
        returnData = dict()
        returnData['info'] = info
        returnData['endPose'] = endPose

        return returnData

    def testROS(self):
        d = self.computeSingleCameraPose()
        self.taskRunner.callOnThread(self.robotService.moveToJointPosition, d['endPose'])

"""
calibrationData is a list of dicts as above
cameraPoses is a CameraPoses object 
"""
def addCameraPosesToDataFile(calibrationData, cameraPoses):
    # augment data with matched camera poses
    # NOTE: camera poses are defined relative to first camera frame, hence the first one will look unintialized
    for index, value in enumerate(calibrationData):
        transform = cameraPoses.getCameraPoseAtUTime(value['utime'])
        (pos, quat) = transformUtils.poseFromTransform(transform)
        calibrationData[index]['camera_frame'] = dict()
        calibrationData[index]['camera_frame']['quaternion'] = dict()
        calibrationData[index]['camera_frame']['quaternion']['w'] = float(quat[0])
        calibrationData[index]['camera_frame']['quaternion']['x'] = float(quat[1])
        calibrationData[index]['camera_frame']['quaternion']['y'] = float(quat[2])
        calibrationData[index]['camera_frame']['quaternion']['z'] = float(quat[3])
        calibrationData[index]['camera_frame']['translation'] = dict()
        calibrationData[index]['camera_frame']['translation']['x'] = float(pos[0])
        calibrationData[index]['camera_frame']['translation']['y'] = float(pos[1])
        calibrationData[index]['camera_frame']['translation']['z'] = float(pos[2])

    return calibrationData

def processCalibrationData(posegraph_file, robot_data_filename, save_data_filename):
    cameraPoses = CameraPoses(posegraph_file)
    calibrationData = spartanUtils.getDictFromYamlFilename(robot_data_filename)
    calibrationData = addCameraPosesToDataFile(calibrationData, cameraPoses)
    spartanUtils.saveToYaml(calibrationData, save_data_filename)

