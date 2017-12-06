# system
import os
import numpy as np
import time

# ROS
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_ros
import rosbag
import actionlib


# spartan ROS
import spartan_grasp_msgs.msg
import spartan_grasp_msgs.srv

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils
from spartan.manipulation.schunk_driver import SchunkDriver

# director
from director import transformUtils
from director import visualization as vis


USING_DIRECTOR = True
if USING_DIRECTOR:
    from spartan.utils.taskrunner import TaskRunner

class BackgroundSubtractionDataCapture(object):

    def __init__(self, jointNames, poseData, cameraInfoDict, cameraSerialNumber=1112170110, poseFilename=None, cameraInfoFilename=None):
        
        self.jointNames = jointNames
        self.poseData = poseData
        self.cameraInfoDict = cameraInfoDict

        self.poseFilename = poseFilename
        self.cameraInfoFilename = cameraInfoFilename
        self.robotService = rosUtils.RobotService(self.jointNames)
        self.cameraSerialNumber = cameraSerialNumber
        self.setupConfig()
        self.setup()

        if USING_DIRECTOR:
            self.taskRunner = TaskRunner()

    def setupConfig(self):
        self.config = dict()
        self.config['image_file_type'] = 'png'
        self.config['maxJointDegreesPerSecond'] = 60

        self.config['encoding'] = dict()
        self.config['encoding']['rgb'] ='bgr8'
        # self.config['encoding']['depth'] = 'mono16'

    def loadConfigFromFiles(self):
        self.poseData = spartanUtils.getDictFromYamlFilename(self.poseFilename)
        self.cameraInfoDict = spartanUtils.getDictFromYamlFilename(self.cameraInfoFilename)

    def setup(self):
        cameraSerialNumber = self.cameraInfoDict['camera_serial_number']
        self.cameraTopicBase = '/camera_' + str(cameraSerialNumber)
        self.imageTopics = dict()
        self.imageTopics['rgb'] = self.cameraTopicBase + '/rgb/image_rect_color'
        self.imageTopics['depth'] = self.cameraTopicBase + '/depth_registered/sw_registered/image_rect'


    def startImageSubscribers(self):
        self.imageSubscribers = dict()
        msgType = sensor_msgs.msg.Image
        for key, topic in self.imageTopics.iteritems():
            sub = rosUtils.SimpleSubscriber(topic, msgType)
            sub.start()
            self.imageSubscribers[key] = sub


    def stopImageSubscribers(self):
        for key, value in self.imageTopics.iteritems():
            self.imageSubscribers[key].stop()


    def setupDataCapture(self, objectName):
        unique_name = time.strftime("%Y%m%d-%H%M%S")
        self.folderName = os.path.join(spartanUtils.getSpartanSourceDir(), 'sandbox','background_subtraction_data', unique_name)
        os.system("mkdir -p " + self.folderName)
        os.chdir(self.folderName)


        self.startImageSubscribers()
        self.data = dict()
        self.data['header'] = dict()
        self.data['header']['object_name'] = objectName
        self.data['images'] = dict()

        for poseName in self.poseData['pose_scan_order']:
            self.data['images'][poseName] = dict()


    def runDataCapture(self, filenameExtension='background'):
        for poseName in self.poseData['pose_scan_order']:
            pose = self.poseData['poses'][poseName]
            self.robotService.moveToJointPosition(pose, maxJointDegreesPerSecond=self.config['maxJointDegreesPerSecond'])

            self.captureImages(poseName, filenameExtension=filenameExtension)


    def captureImages(self, poseName, filenameExtension='background'):
        d = dict()
        
        for imageType, topic in self.imageTopics.iteritems():

            filename = str(poseName) + "_" + imageType + "_" + filenameExtension + '.' + self.config['image_file_type']
            fullFilename =  os.path.join(self.folderName, filename)

            # use the specified encoding if it is given
            encoding = None
            if imageType in self.cameraInfoDict['encoding']:
                encoding = self.cameraInfoDict['encoding'][imageType]

            rosUtils.saveSingleImage(topic, fullFilename, encoding)

            d[imageType] = dict()
            d[imageType]['filename'] = filename


        self.data['images'][poseName][filenameExtension] = d
        

    def saveData(self):
        filename = os.path.join(self.folderName, 'data.yaml')
        spartanUtils.saveToYaml(self.data, filename)

    def test(self):
        self.taskRunner.callOnThread(self.setupDataCapture, 'oil_bottle')
        time.sleep(3)
        self.taskRunner.callOnThread(self.runDataCapture, 'background')

    def testForeground(self):
        self.taskRunner.callOnThread(self.runDataCapture, 'foreground')

    @staticmethod
    def makeDefault():
        storedPosesFile = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects', 'station_config','RLG_iiwa_1','background_subtraction', 'stored_poses.yaml')

        cameraInfoFilename =  os.path.join(spartanUtils.getSpartanSourceDir(), 'src/catkin_projects/camera_config/data/1112170110/master/camera_ros_data.yaml')

        poseData = spartanUtils.getDictFromYamlFilename(storedPosesFile)
        cameraInfoDict = spartanUtils.getDictFromYamlFilename(cameraInfoFilename)


        return BackgroundSubtractionDataCapture(poseData['header']['joint_names'], poseData, cameraInfoDict, poseFilename=storedPosesFile, cameraInfoFilename=cameraInfoFilename)
