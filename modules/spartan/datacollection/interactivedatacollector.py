#system
import numpy as np
import time
import os
import subprocess
import random

#director
from director import transformUtils
from director import robotsystem
from director import roboturdf
from director import visualization as vis
from director import ikplanner
from director import ikconstraints
from director import robotstate
from director import segmentation
from director.ikplanner import ConstraintSet
import director.vtkAll as vtk
from director.debugVis import DebugData
from director import objectmodel as om
from director.ikparameters import IkParameters
from director import filterUtils
from director import vtkNumpy
RobotPoseGUIWrapper = ikplanner.RobotPoseGUIWrapper

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils
from spartan.utils.taskrunner import TaskRunner

# drake
import drake as lcmdrake
from director import lcmUtils

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

    def sendGripperCommand(self, targetPositionMM, force):
        msg = lcmdrake.lcmt_schunk_wsg_command()
        msg.utime = int(time.time()*1e6)
        msg.force = force
        msg.target_position_mm = targetPositionMM
        lcmUtils.publish('SCHUNK_WSG_COMMAND', msg)


    def gripperOpen(self):
        self.sendGripperCommand(100, 40)


    def gripperClose(self):
        self.sendGripperCommand(0, 40)


class InteractiveDataCollector(object):

    def __init__(self, robotSystem, handFrame='palm', pointCloud=None, cameraTransform=None):
        self.robotSystem = robotSystem
        self.robotService = RobotService(robotSystem)
        self.handFrame = handFrame
        self.setupConfig()
        self.taskRunner = TaskRunner()
        self.visFolder = om.getOrCreateContainer('interactive data collector')
        if pointCloud:
            self.pointCloud = pointCloud
        if cameraTransform:
            self.cameraTransform = cameraTransform

    def segmentPointCloud(self):
        pointAboveTable = np.array([0.7, 0, 0.4])
        pointOnTable = np.array([0.7, 0, 0.2])
        expectedNormal = pointAboveTable - pointOnTable
        expectedNormal = expectedNormal/np.linalg.norm(expectedNormal)

        pointCloudPolyData = self.pointCloud.polyData
        transformedPointCloudPolyData = filterUtils.transformPolyData(pointCloudPolyData, self.cameraTransform.getCameraToWorld())
        polyData, normal = segmentation.applyPlaneFit(pointCloudPolyData, searchOrigin=pointOnTable, searchRadius=0.3, expectedNormal=expectedNormal)

        # get points above plane
        thickness = 0.01
        abovePolyData = filterUtils.thresholdPoints(polyData, 'dist_to_plane', [thickness / 2.0, np.inf])
        belowPolyData = filterUtils.thresholdPoints(polyData, 'dist_to_plane', [-np.inf, -thickness / 2.0])
        
        # crop to sphere
        croppedPolyData = segmentation.cropToSphere(abovePolyData, self.tableTopPosesConfig['table_center'], 0.4)

        # some debugging visualization
        visualize = True
        if visualize:
            vis.showPolyData(abovePolyData, 'above table segmentation', color=[0, 1, 0],
                             parent=self.visFolder)

            vis.showPolyData(pointCloudPolyData, 'all point cloud', color=[1, 0, 0],
                             parent=self.visFolder)

            vis.showPolyData(croppedPolyData, 'cropped to Sphere', color=[0, 0, 1],
                             parent=self.visFolder)

        self.croppedPolyData = croppedPolyData


    def graspHighestPoint(self):
        self.segmentPointCloud()
        numpyPointCloud = vtkNumpy.getNumpyFromVtk(self.croppedPolyData)
        highestPointIndex = np.argmax(numpyPointCloud[:,2])
        highestPoint = numpyPointCloud[highestPointIndex]
        # hack to effectively transform to gripper point
        highestPoint = highestPoint + np.array([0.03, 0.0, 0.14])
        downwardRotation = np.array([0.0,0,1.0,0.0])
        downwardRotation = downwardRotation/np.linalg.norm(downwardRotation)
        ikResult = self.computeSingleCameraPose(cameraFrameLocation=highestPoint, quaternionDesired=downwardRotation)
        
        if (ikResult['info'] == 1):
            rospy.loginfo("\n grasping")
            self.robotService.moveToJointPosition(ikResult['endPose'])
            time.sleep(1)
            result = dict()
            result['joint_angles'] = ikResult['endPose']
            result['cameraLocation'] = highestPoint
            return result, True
        else:
            rospy.loginfo("\n could not grasp")
            return None, False

    def setupConfig(self):
        self.config = dict()
        self.config['rgb_raw_topic'] = '/camera/rgb/image_raw'

        config = dict()

        config['direction_to_target'] = [1, 0, 0]   # this should be in the x,y plane
        config['table_center'] = [0.7, 0, 0.3]        # in world frame, where world origin is robot base

        config['table_width'] = 0.5
        config['table_width_step_size'] = 0.05

        config['table_depth'] = 0.2
        config['table_depth_step_size'] = 0.025

        self.tableTopPosesConfig = config

    def getTableHeight(self):
        return self.tableTopPosesConfig['table_center'][2]

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


    def iiwaJointDictToList(self, iiwa_joint_dict):
        joint_angles_list = []
        for i in range(7):
            joint_key = 'iiwa_joint_' + str(i+1)
            joint_angles_list.append(iiwa_joint_dict[joint_key])
        return joint_angles_list


    def testGripper(self):
        # close gripper
        self.robotService.gripperClose()
        time.sleep(1)

        # open gripper
        self.robotService.gripperOpen()
        time.sleep(1)


    """
    Warning: Don't call this function directly, use run() instead, which
    calls this function in a thread
    """
    def runROSDataCollect(self):
        topicName = None
        self.passiveSubscriber = None
        if self.captureRGB:
            topicName = self.config['rgb_raw_topic']

        if topicName:
            self.passiveSubscriber = spartanROSUtils.SimpleSubscriber(topicName, sensor_msgs.msg.Image)
            self.passiveSubscriber.start()

        unique_name = time.strftime("%Y%m%d-%H%M%S")
        self.calibrationFolderName = os.path.join(spartanUtils.getSpartanSourceDir(), 'auto_interactive_data', unique_name)
        os.system("mkdir -p " + self.calibrationFolderName)
        os.chdir(self.calibrationFolderName)

        poseDict = self.computeTableTopPoses()
        self.drawResult(poseDict)
        rospy.loginfo("finished making calibration poses")
        rospy.loginfo("starting calibration run")

        autoCollectedData = []

        num_interactions = 1
        for i in range(num_interactions):

            # pick towel-picking pose
            pose, success = self.graspHighestPoint()
            if not success:
                pose = random.choice(poseDict['feasiblePoses'])
                print "1", pose['joint_angles']
                rospy.loginfo("\n moving to pose")
                self.robotService.moveToJointPosition(pose['joint_angles'])

            # close gripper
            self.robotService.gripperClose()
            time.sleep(1)

            # flick motion
            self.performFlickMotion(currentPose=pose)

            # open gripper
            self.robotService.gripperOpen()
            time.sleep(1)

            # move back home
            homePose = RobotPoseGUIWrapper.getPose('Elastic Fusion', 'home')
            print "2", homePose
            homeJointAngles = self.iiwaJointDictToList(homePose)
            rospy.loginfo("\n moving to home")
            self.robotService.moveToJointPosition(homeJointAngles)

            # save data
            rospy.loginfo("capturing images and robot data")
            data = self.captureCurrentRobotAndImageData(captureRGB=self.captureRGB)
            autoCollectedData.append(data)

        rospy.loginfo("finished data collection routine, saving data to file")

        spartanUtils.saveToYaml(autoCollectedData, os.path.join(self.calibrationFolderName, 'robot_data.yaml'))

        if self.passiveSubscriber:
            self.passiveSubscriber.stop()

        return autoCollectedData

    def performFlickMotion(self, currentPose):
        # create random flick vector
        flickMotionX = random.uniform(-0.1, 0.1)
        flickMotionY = random.uniform(-0.1, 0.1)
        flickMotionZ = random.uniform(0.02, 0.2)
        flickMotionVector = np.array([flickMotionX, flickMotionY, flickMotionZ])

        flickFinalPosition = np.array(currentPose['cameraLocation']) + flickMotionVector
        ikResult = self.computeSingleCameraPose(cameraFrameLocation=flickFinalPosition)
        
        if (ikResult['info'] == 1):
            rospy.loginfo("\n flicking")
            self.robotService.moveToJointPosition(ikResult['endPose'])
            time.sleep(1)
        else:
            rospy.loginfo("\n could not flick")


    def run(self, captureRGB=False):
        self.captureRGB = captureRGB
        self.taskRunner.callOnThread(self.runROSDataCollect)


    def computeSingleTablePosition(self, table_center, x_position, y_position):
        x = table_center[0] + x_position
        y = table_center[1] + y_position
        z = table_center[2]
        return [x, y, z]


    def computeTableTopPoses(self):

        config = self.tableTopPosesConfig

        n_y= int(np.ceil(config['table_width']/config['table_width_step_size']))
        y_samples = np.linspace(-config['table_width']/2, config['table_width']/2, n_y)

        n_x = int(np.ceil(config['table_depth']/config['table_depth_step_size']))
        x_samples = np.linspace(-config['table_depth']/2, config['table_depth']/2, n_x)

        print 'x_samples', x_samples
        print 'y_samples', y_samples

        tableTopPoses = []
        returnData = dict()
        returnData['x_samples'] = x_samples
        returnData['y_samples'] = y_samples
        returnData['cameraLocations'] = []
        returnData['feasiblePoses'] = []

        for x_position in x_samples:
            for y_position in y_samples:
                
                cameraLocation = self.computeSingleTablePosition(config['table_center'], x_position, y_position)
                ikResult = self.computeSingleCameraPose(cameraFrameLocation=cameraLocation)
                returnData['cameraLocations'].append(cameraLocation)

                if (ikResult['info'] == 1):
                    d = dict()
                    d['joint_angles'] = ikResult['endPose']
                    d['cameraLocation'] = cameraLocation
                    returnData['feasiblePoses'].append(d)

                    print "info = ", ikResult['info']
                    print ""
                    tableTopPoses.append(ikResult['endPose'])

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

    @staticmethod
    def createRotationConstraint(quaternion, linkName=None):

        p = ikconstraints.QuatConstraint()
        p.linkName = linkName
        p.angleToleranceInDegrees = 5.0
        p.quaternion = np.array(quaternion)

        return p

    def computeSingleCameraPose(self, cameraFrameLocation=[0.22, 0, 0.89], quaternionDesired=None):
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

        positionConstraint = InteractiveDataCollector.createPositionConstraint(targetPosition=cameraFrameLocation, linkName=linkName, linkOffsetFrame=cameraToLinkFrame, positionTolerance=0.05)
        constraints.append(positionConstraint)

        if quaternionDesired is not None:
            rotationConstraint = InteractiveDataCollector.createRotationConstraint(quaternionDesired, linkName=linkName)
            constraints.append(rotationConstraint)

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
        result = self.computeTableTopPoses()
        self.drawResult(result)