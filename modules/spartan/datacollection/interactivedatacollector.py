#system
import numpy as np
import time
import os
import subprocess

#director
from director import transformUtils
from director import robotsystem
from director import roboturdf
from director import visualization as vis
from director import ikplanner
from director import ikconstraints
from director import robotstate
from director.ikplanner import ConstraintSet
import director.vtkAll as vtk
from director.debugVis import DebugData
from director import objectmodel as om
from director.ikparameters import IkParameters

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils
from spartan.utils.taskrunner import TaskRunner

# ROS
import rospy
import sensor_msgs.msg

# ROS custom
import robot_msgs.srv


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

    def getPose(self):
        q = self.robotSystem.robotStateJointController.q
        if self.removeFloatingBase:
            q = q[6:]

        return q


class InteractiveDataCollector(object):

    def __init__(self, robotSystem, handFrame='palm'):
        self.robotSystem = robotSystem
        self.robotService = RobotService(robotSystem)
        self.handFrame = handFrame
        self.setupConfig()
        self.taskRunner = TaskRunner()


    def setupConfig(self):
        self.config = dict()
        self.config['rgb_raw_topic'] = '/camera/rgb/image_raw'

        config = dict()
        config['yaw'] = dict()
        config['yaw']['min'] = -180
        config['yaw']['max'] = 180
        config['yaw']['step_size'] = 5

        config['pitch'] = dict()
        config['pitch']['min'] = 10
        config['pitch']['max'] = 60
        config['pitch']['step_size'] = 5

        config['distance'] = dict()
        config['distance']['min'] = 0.60
        config['distance']['max'] = 1.3
        config['distance']['step_size'] = 0.4

        config['direction_to_target'] = [1, 0, 0] # this should be in the x,y plane
        config['target_location'] = [0.75, 0, 0]

        self.calibrationPosesConfig = config


    def captureDataAtPose(self, poseName=""):
        data = dict()
        data['pose_name'] = poseName
        data['pose'] = self.robotService.getPose().tolist()
        data['hand_frame_name'] = self.handFrame
        handTransform = self.robotSystem.robotStateModel.getLinkFrame(self.handFrame)
        data['hand_frame'] = spartanUtils.poseFromTransform(handTransform)
        data['utime'] = self.robotSystem.robotStateJointController.lastRobotStateMessage.utime
        return data

    def saveSingleImage(self, topic, filename, encoding):
        rosImageLoggerExecutable = os.path.join(spartanUtils.getSpartanSourceDir(), 'modules',"spartan",
                                                'calibration','ros_image_logger.py')
        cmd = "%s -t %s -f %s -e %s" % (rosImageLoggerExecutable, topic, filename, encoding)
        os.system(cmd)

    def captureCurrentRobotAndImageData(self, captureRGB=False):
        data = dict()
        data['joint_positions'] = self.robotService.getPose().tolist()
        data['hand_frame_name'] = self.handFrame

        handTransform = self.robotSystem.robotStateModel.getLinkFrame(self.handFrame)
        data['hand_frame'] = spartanUtils.poseFromTransform(handTransform)

        rosTime = rospy.Time.now()
        data['ros_timestamp'] = rosTime.to_sec()


        data['images'] = dict()
        imgTopics = dict()
        if captureRGB:
            imgTopics['rgb'] = self.config['rgb_raw_topic']

        
        for key, topic in imgTopics.iteritems():

            imageFilename =str(data['ros_timestamp']) + "_" + key + ".jpeg"
            fullImageFilename = os.path.join(self.calibrationFolderName, imageFilename)

            encoding = None
            if key == "rgb":
                encoding = 'bgr8'
            elif key == "ir":
                encoding = 'mono16'
            else:
                raise ValueError("I only know how to handle ir and rgb")


            self.saveSingleImage(topic, fullImageFilename, encoding)

            singleImgData = dict()
            singleImgData['filename'] = imageFilename
            data['images'][key] = singleImgData


        return data

    """
    Warning: Don't call this function directly, use run() instead, which
    calls this function in a thread
    """
    def runROSCalibration(self, headerData):

        headerData['target']['location_estimate_in_robot_base_frame'] = self.calibrationPosesConfig['target_location'] 
        calibrationRunData = dict()
        calibrationRunData['header'] = headerData

        #setup our passive subscribers to the IR or RGB data
        topicName = None
        self.passiveSubscriber = None
        if self.captureRGB:
            topicName = self.config['rgb_raw_topic']

        if topicName:
            self.passiveSubscriber = spartanROSUtils.SimpleSubscriber(topicName, sensor_msgs.msg.Image)
            self.passiveSubscriber.start()

        unique_name = time.strftime("%Y%m%d-%H%M%S")
        self.calibrationFolderName = os.path.join(spartanUtils.getSpartanSourceDir(), 'calibration_data', unique_name)
        os.system("mkdir -p " + self.calibrationFolderName)
        os.chdir(self.calibrationFolderName)

        poseDict = self.computeCalibrationPoses()
        self.drawResult(poseDict)
        rospy.loginfo("finished making calibration poses")
        rospy.loginfo("starting calibration run")

        calibrationData = []


        for pose in poseDict['feasiblePoses']:
            # move robot to that joint position
            rospy.loginfo("\n moving to pose")
            self.robotService.moveToJointPosition(pose['joint_angles'])

            rospy.loginfo("capturing images and robot data")
            data = self.captureCurrentRobotAndImageData(captureRGB=self.captureRGB)
            calibrationData.append(data)

        rospy.loginfo("finished calibration routine, saving data to file")

        
        calibrationRunData['data_list'] = calibrationData

        spartanUtils.saveToYaml(calibrationRunData, os.path.join(self.calibrationFolderName, 'robot_data.yaml'))

        if self.passiveSubscriber:
            self.passiveSubscriber.stop()

        return calibrationRunData

    def run(self, captureRGB=True, cameraName="xtion pro", targetWidth=4,
        targetHeight=5, targetSquareSize=0.05):

        self.captureRGB = captureRGB
        
        # setup header information for storing along with the log
        calibrationHeaderData = dict()
        calibrationHeaderData['camera'] = cameraName
        
        targetData = dict()
        targetData['width'] = targetWidth
        targetData['height'] = targetHeight
        targetData['square_edge_length'] = targetSquareSize

        calibrationHeaderData['target'] = targetData

        self.taskRunner.callOnThread(self.runROSCalibration, calibrationHeaderData)

    def computeCalibrationPoses(self):

        config = self.calibrationPosesConfig

        def arrayFromConfig(d):
            n = int(np.ceil((d['max'] - d['min'])/d['step_size']))
            return np.linspace(d['min'], d['max'], n)

        # vector going from the target back towards the robot

        yawAngles = arrayFromConfig(config['yaw'])
        pitchAngles = arrayFromConfig(config['pitch'])
        distances = arrayFromConfig(config['distance'])

        calibrationPoses = []
        returnData = dict()
        returnData['yawAngles'] = yawAngles
        returnData['pitchAngles'] = pitchAngles
        returnData['cameraLocations'] = []
        returnData['feasiblePoses'] = []

        for dist in distances:
            for pitch in pitchAngles:
                for yaw in yawAngles:
                    # if (pitch > 70) and (dist < 0.8):
                    #     print "skipping pose"
                    #     continue

                    cameraLocation = HandEyeCalibration.gripperPositionTargetRadialFromCalibrationPlate(config['target_location'], yaw=yaw, pitch=pitch, radius=dist)

                    ikResult = self.computeSingleCameraPose(cameraFrameLocation=cameraLocation, targetLocationWorld=config['target_location'])

                    returnData['cameraLocations'].append(cameraLocation)


                    if (ikResult['info'] == 1):
                        d = dict()
                        d['yaw'] = yaw
                        d['pitch'] = pitch
                        d['dist'] = dist
                        d['joint_angles'] = ikResult['endPose']
                        d['cameraLocation'] = cameraLocation
                        returnData['feasiblePoses'].append(d)


                        print "\n yaw %.2f, pitch %.2f, distance %.2f" % (yaw, pitch, dist)
                        print "info = ", ikResult['info']
                        print ""
                        calibrationPoses.append(ikResult['endPose'])


        return returnData



    def drawResult(self, result):

        radius = 0.01
        parent = om.getOrCreateContainer('Hand Eye Calibration')

        d = DebugData()
        for cameraLocation in result['cameraLocations']:
            d.addSphere(cameraLocation, radius=radius)


        vis.updatePolyData(d.getPolyData(), 'All Camera Locations', parent=parent, color=[1,0,0])

        d = DebugData()
        for data in result['feasiblePoses']:
            d.addSphere(data['cameraLocation'], radius=radius)

        vis.updatePolyData(d.getPolyData(), 'Feasible Camera Locations', parent=parent, color=[0,1,0])

    @staticmethod
    def gripperPositionTargetRadialFromCalibrationPlate(calibrationPlateLocation, yaw=None, pitch=None, roll=0, radius=None):

        calibrationPlateLocation = np.array(calibrationPlateLocation)

        theta = np.deg2rad(pitch)
        phi = np.deg2rad(yaw)
        v = np.zeros(3)
        v[0] = np.sin(theta) * np.cos(phi)
        v[1] = np.sin(theta) * np.sin(phi)
        v[2] = np.cos(theta)

        v = v/np.linalg.norm(v)

        s = radius * v + calibrationPlateLocation

        return s


    @staticmethod
    def createCameraGazeTargetConstraint(linkName=None, cameraToLinkFrame=None, cameraAxis=None, worldPoint=None, coneThresholdDegrees=0.0):

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
        constraints.append(cameraGazeConstraint)

        constraintSet = ConstraintSet(ikPlanner, constraints, 'reach_end',
                                      startPoseName)
        constraintSet.ikParameters = IkParameters()

        constraintSet.seedPoseName = seedPoseName

        endPose, info = constraintSet.runIk()
        returnData = dict()
        returnData['info'] = info
        returnData['endPose'] = endPose

        return returnData

    def testComputePoses(self):
        result = self.computeCalibrationPoses()
        self.drawResult(result)