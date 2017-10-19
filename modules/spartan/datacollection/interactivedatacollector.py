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
from director.ikplanner import ConstraintSet
import director.vtkAll as vtk
from director.debugVis import DebugData
from director import objectmodel as om
from director.ikparameters import IkParameters
RobotPoseGUIWrapper = ikplanner.RobotPoseGUIWrapper

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

        num_interactions = 10
        for i in range(num_interactions):

            # pick random feasible pose
            pose = random.choice(poseDict['feasiblePoses'])

            # move robot to that joint position
            print "1", pose['joint_angles']
            rospy.loginfo("\n moving to pose")
            self.robotService.moveToJointPosition(pose['joint_angles'])

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

    def computeSingleCameraPose(self, cameraFrameLocation=[0.22, 0, 0.89]):
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