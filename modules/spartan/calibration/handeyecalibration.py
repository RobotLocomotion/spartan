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
from director.debugVis import DebugData
from director import objectmodel as om




RobotPoseGUIWrapper = ikplanner.RobotPoseGUIWrapper
import bot_core as lcmbotcore
from director.ikparameters import IkParameters


# try:
#     from labelfusion.cameraposes import CameraPoses
# except:
#     print "Have you sourced setup_environment_labelfusion.sh in spartan/config?"
#     quit()

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils
from spartan.utils.taskrunner import TaskRunner


# ROS
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2

# ROS custom
import robot_msgs.srv


"""
To run this set useKukaRLGDev and useROS to True in iiwaManipApp.py. This loads a
HandEyeCalibration object into the director workspace, it is called cal.

To run calibration simply press F8 and enter cal.run(). This creates a new directory
in spartan/calibration_data which contains two files.

robot_data.yaml:
    - relevant data for the "hand_link" poses during the run.
    - filenames corresponding to images that were saved

You must calibrate IR and RGB in separate runs as the ROS driver cannot stream both simultaneously. 
For example first run cal.run(createRGB=True, createIR=False) then
cal.run(createRGB=False, createIR=True).    



DEPRECATED

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

    def __init__(self, robotSystem, handFrame='wsg_50_base_link', cameraSerialNumber="", configFilename=None):
        self.robotSystem = robotSystem
        self.configFilename = configFilename
        self.robotService = RobotService(robotSystem)
        self.handFrame = handFrame
        self.cameraSerialNumber = cameraSerialNumber
        self.setup()
        self.calibrationData = None
        self.setupConfig()
        

        self.timer = TimerCallback(targetFps=1)
        self.timer.callback = self.callback
        self.taskRunner = TaskRunner()
    # self.timer.callback = self.callback

    def setupConfig(self):
        self.config = dict()
        self.config['rgb_raw_topic'] = '/camera' + str(self.cameraSerialNumber) + '/rgb/image_raw'
        self.config['ir_raw_topic'] = '/camera' + str(self.cameraSerialNumber) + '/ir/image'

        config = dict()
        config['yaw'] = dict()
        config['yaw']['min'] = -180
        config['yaw']['max'] = 180
        config['yaw']['step_size'] = 15

        config['pitch'] = dict()
        config['pitch']['min'] = 10
        config['pitch']['max'] = 60
        config['pitch']['step_size'] = 10

        config['distance'] = dict()
        config['distance']['min'] = 0.60
        config['distance']['max'] = 1.3
        config['distance']['step_size'] = 0.2

        config['direction_to_target'] = [1, 0, 0] # this should be in the x,y plane
        config['target_location'] = [0.6, 0, 0]

        self.config['poses'] = config

        self.calibrationPosesConfig = config


    def loadConfigFromFile(self, filename=None):
        if filename is None:
            filename = self.configFilename

        self.config = spartanUtils.getDictFromYamlFilename(filename)
        self.calibrationPosesConfig = self.config['poses']

    # DEPRECATED
    # # This function inits a simple subscriber node to passively listen to the recorded image topics
    # # This ensures that saving .jpg images is not corrupted
    # def initSimpleSubscriber(self):
    #     rosSimpleSubscriberExecutable = os.path.join(spartanUtils.getSpartanSourceDir(), 'modules',"spartan",
    #                                             'utils','simple_subscriber.py')
    #     cmd  = "%s --topic-package-type " % (rosSimpleSubscriberExecutable)
    #     cmd += " ['%s','%s','%s'] " % (self.config['rgb_raw_topic'], 'sensor_msgs.msg', 'Image')
    #     cmd += " ['%s','%s','%s'] " % (self.config['ir_raw_topic'], 'sensor_msgs.msg', 'Image')
    #     cmd += " &"    # this node needs to be backgrounded
    #     print "CMD IS", cmd
    #     os.system(cmd)

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

    def getImages(self, captureRGB=True, captureIR=True):
        d = dict()
        if captureRGB:
            d['rgb'] = self.getSingleImage(self.config['rgb_raw_topic'])

        if captureIR:
            d['ir'] = self.getSingleImage(self.config['ir_raw_topic'])


    def getSingleImage(self, topic):
        rospy.loginfo("waiting for image on topic %s", topic)
        # bridge = CvBridge()
        d = dict()
        msg = self.subscribers[topic].waitForNextMessage()
        rospy.loginfo("received message on topic %s", topic)
        print "type(msg) ", type(msg)

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError, e:
            print(e)

        d['msg'] = msg
        d['cv2_img'] = cv2_img

        rospy.loginfo("converted msg to cv2 img on topic %s", topic)

        return d

    def saveSingleImage(self, topic, filename, encoding):
        rosImageLoggerExecutable = os.path.join(spartanUtils.getSpartanSourceDir(), 'modules',"spartan",
                                                'calibration','ros_image_logger.py')
        cmd = "%s -t %s -f %s -e %s" % (rosImageLoggerExecutable, topic, filename, encoding)
        os.system(cmd)

    def captureCurrentRobotAndImageData(self, captureRGB=False, captureIR=False):
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

        if captureIR:
            imgTopics['ir'] = self.config['ir_raw_topic']



        
        for key, topic in imgTopics.iteritems():

            imageFilename =str(data['ros_timestamp']) + "_" + key + "." + self.config['filename_extension']
            fullImageFilename = os.path.join(self.calibrationFolderName, imageFilename)

            encoding = None
            if key == "rgb":
                encoding = self.config['encoding'][key]
            elif key == "ir":
                encoding = self.config['encoding'][key]
            else:
                raise ValueError("I only know how to handle ir and rgb")



            self.saveSingleImage(topic, fullImageFilename, encoding)

            singleImgData = dict()
            singleImgData['filename'] = imageFilename
            data['images'][key] = singleImgData


        return data


    def testCaptureData(self):
        data = self.captureDataAtPose()
        self.calibrationData.append(data)

    def moveHome(self):
        self.robotService.movePose(self.poseDict['center']['nominal'])

    def runThreaded(self):
        self.taskRunner.callOnThread(self.runLCMCalibration)

    def runLCMCalibration(self):
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
                    continue
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


    # DEPRECATED
    # def runCalibration(self):
    #     self.calibrationData = []
    #     unique_name = time.strftime("%Y%m%d-%H%M%S") + "_" + self.calibrationType
    #     unique_name = str(unique_name) + "_" + self.calibrationType
    #     self.calibrationFolderName = os.path.join(spartanUtils.getSpartanSourceDir(), 'calibration_data',unique_name)
    #     print "calibration folder name ", self.calibrationFolderName
    #     os.system("mkdir -p " + self.calibrationFolderName)
    #     os.chdir(self.calibrationFolderName)
    #     cmd = "lcm-logger calibration.lcmlog"
    #     self.loggerProcess = subprocess.Popen("exec " + cmd, shell = True)

    #     # waiting for logger to start
    #     print "waiting for logger to start . . . "
    #     time.sleep(2.0) # give the logger process time to start

    #     self.makePoseOrder()
    #     self.timer.start()

    def test(self):
        self.taskRunner.callOnThread(self.getImages, True, False)

    def saveCalibrationData(self, filename=None):
        if filename is None:
            filename = os.path.join(spartanUtils.getSpartanSourceDir(), 'sandbox', 'hand_eye_calibration_robot_data.yaml')


        spartanUtils.saveToYaml(self.calibrationData, filename)

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

        if self.captureIR:
            topicName = self.config['ir_raw_topic']

        if topicName:
            self.passiveSubscriber = spartanROSUtils.SimpleSubscriber(topicName, sensor_msgs.msg.Image)
            self.passiveSubscriber.start()

        unique_name = time.strftime("%Y%m%d-%H%M%S") + "_" + self.calibrationType
        self.calibrationFolderName = os.path.join(spartanUtils.getSpartanSourceDir(), 'calibration_data', unique_name)
        os.system("mkdir -p " + self.calibrationFolderName)
        os.chdir(self.calibrationFolderName)

        poseDict = self.computeCalibrationPoses()
        self.drawResult(poseDict)
        rospy.loginfo("finished making calibration poses")
        rospy.loginfo("starting calibration run")

        calibrationData = []

        # topic = self.config['rgb_raw_topic']
        # msgType = sensor_msgs.msg.Image
        # self.subscribers = dict()
        # self.subscribers[topic] = spartanROSUtils.SimpleSubscriber(topic, msgType)
        # self.subscribers[topic].start()



        for pose in poseDict['feasiblePoses']:
            # move robot to that joint position
            rospy.loginfo("\n moving to pose")
            self.robotService.moveToJointPosition(pose['joint_angles'])

            rospy.loginfo("capturing images and robot data")
            data = self.captureCurrentRobotAndImageData(captureRGB=self.captureRGB, captureIR=self.captureIR)
            calibrationData.append(data)

        rospy.loginfo("finished calibration routine, saving data to file")

        
        calibrationRunData['data_list'] = calibrationData

        spartanUtils.saveToYaml(calibrationRunData, os.path.join(self.calibrationFolderName, 'robot_data.yaml'))

        if self.passiveSubscriber:
            self.passiveSubscriber.stop()

        self.moveHome()

        return calibrationRunData

    def run(self, captureRGB=True, captureIR=False, cameraName="sr300", targetWidth=4,
        targetHeight=5, targetSquareSize=0.05):

        if cameraName=="xtion pro" and captureRGB and captureIR:
            print "you can't capture IR and RGB at the same time, returning"
            return

        self.captureRGB = captureRGB
        self.captureIR = captureIR

        self.calibrationType = None
        
        if self.captureRGB:
            self.calibrationType = 'rgb'
        elif self.captureIR:
            self.calibrationType = 'ir'
        else:
            self.calibrationType = "no_images"


        
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

        previousCameraLocation = None

        for dist in distances:
            for pitch in pitchAngles:
                for yaw in yawAngles:
                    # if (pitch > 70) and (dist < 0.8):
                    #     print "skipping pose"
                    #     continue

                    # cameraLocation = HandEyeCalibration.gripperPositionTarget(config['target_location'], yaw=yaw, pitch=pitch, radius=dist)
                    cameraLocation = HandEyeCalibration.gripperPositionTargetRadialFromCalibrationPlate(config['target_location'], yaw=yaw, pitch=pitch, radius=dist)


                    # if it's too close to previous one, then skip it
                    if previousCameraLocation is not None:
                        if np.linalg.norm(previousCameraLocation - cameraLocation) < self.config['poses']['min_distance_between_poses']:
                            print("skipping pose, too close to previous pose")
                            continue

                    previousCameraLocation = cameraLocation

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
    def gripperPositionTarget(directionToTarget, yaw=None, pitch=None, roll=0, radius=None):

        # project into x,y plane
        d = np.array(directionToTarget)
        d = d/np.linalg.norm(d)
        d[2] = 0

        theta = np.deg2rad(pitch)
        phi = np.deg2rad(yaw)
        v = np.zeros(3)
        v[0] = np.sin(theta) * np.cos(phi)
        v[1] = np.sin(theta) * np.sin(phi)
        v[2] = np.cos(theta)

        v = v/np.linalg.norm(v)

        # transform that takes [1,0,0] to v
        # t = transformUtils.getTransformFromOriginAndNormal([0,0,0], v)


        # d = np.array(directionToTarget)
        # d = d/np.linalg.norm(d)
        # t = transformUtils.getTransformFromOriginAndNormal([0,0,0], d, normalAxis=0)
        # s = radius * np.array(t.TransformVector(v))

        s = radius * v

        # rotate s if needed
        # alpha = angle between [1,0,0] and d
        xAxis = np.array([1,0,0])
        alpha = np.arccos(np.dot(xAxis, d))
        alphaDegrees = np.rad2deg(alpha)
        t = vtk.vtkTransform()
        t.RotateZ(alphaDegrees)
        xAxisPlus = np.array(t.TransformVector(xAxis))

        t2 = vtk.vtkTransform()
        t2.RotateZ(-alphaDegrees)
        xAxisMinus = np.array(t2.TransformVector(xAxis))

        if np.dot(xAxisPlus, d) > np.dot(xAxisMinus, d):
            s = t.TransformVector(s)
        else:
            s = t2.TransformVector(s)

        
        return s

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

        # WorldGazeTargetConstraint can't be parsed by pydrakeik.py so we need to use WorldGazeDirConstraint

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
    def createCameraGazeDirConstraint(linkName=None, cameraToLinkFrame=None, cameraAxis=None, worldPoint=None,
                                         coneThresholdDegrees=0.0):

        # WorldGazeTargetConstraint can't be parsed by pydrakeik.py so we need to use WorldGazeDirConstraint

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
        cameraAxis = [0,0,1]

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

    def testROS(self):
        topic = self.config['rgb_raw_topic']
        msgType = sensor_msgs.msg.Image
        self.simpleSubscriber = spartanROSUtils.SimpleSubscriber(topic, msgType)
        self.taskRunner.callOnThread(self.simpleSubscriber.start)

        topic = "/joint_states"
        msgType = sensor_msgs.msg.JointState
        self.jointStatesSubscriber = spartanROSUtils.SimpleSubscriber(topic, msgType)
        self.taskRunner.callOnThread(self.jointStatesSubscriber.start)


    def testImageLogger(self, filename = "test.jpeg"):
        fullFilename = os.path.join('/home/manuelli/sandbox', filename)
        topic = self.config['rgb_raw_topic']
        cmd = "ros_image_logger.py -t %s -f %s" %(topic, fullFilename)
        self.taskRunner.callOnThread(os.system, cmd)

    def testComputePoses(self):
        result = self.computeCalibrationPoses()
        self.drawResult(result)
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

