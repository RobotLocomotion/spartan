# system
import os

# ROS
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_ros
import rosbag

# spartan ROS
import spartan_grasp_msgs.msg
import spartan_grasp_msgs.srv

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils

# director
from director import transformUtils
from director import visualization as vis


USING_DIRECTOR = True
if USING_DIRECTOR:
    from spartan.utils.taskrunner import TaskRunner


class GraspSupervisor(object):

    def __init__(self, storedPosesFile=None, cameraSerialNumber=1112170110):
        self.storedPoses = spartanUtils.getDictFromYamlFilename(storedPosesFile)
        self.cameraSerialNumber = cameraSerialNumber

        self.cameraName = 'camera_' + str(cameraSerialNumber)
        self.pointCloudTopic = '/' + str(self.cameraName) + '/depth/points'
        self.graspFrameName = 'base'
        self.depthOpticalFrameName = self.cameraName + "_depth_optical_frame"

        self.robotService = rosUtils.RobotService(self.storedPoses['header']['joint_names'])

        self.usingDirector = True
        self.setupConfig()

        if USING_DIRECTOR:
            self.taskRunner = TaskRunner()
            self.taskRunner.callOnThread(self.setupSubscribers)
            self.taskRunner.callOnThread(self.setupTF)
        else:
            self.setupSubscribers()
            self.setupTF()

    def setupConfig(self):
        self.config = dict()
        self.config['scan'] = dict()
        self.config['scan']['pose_list'] = ['scan_left', 'scan_right']
        self.config['scan']['joint_speed'] = 60
        self.config['grasp_speed'] = 20
        self.config['home_pose_name'] = 'above_table_pre_grasp'
        self.config['grasp_to_ee'] = dict()

        self.config['grasp_to_ee']['translation'] = dict()
        self.config['grasp_to_ee']['translation']['x'] = 9.32362425e-02
        self.config['grasp_to_ee']['translation']['y'] = 0
        self.config['grasp_to_ee']['translation']['z'] = 0

        self.config['grasp_to_ee']['orientation'] = dict()
        self.config['grasp_to_ee']['orientation']['w'] = 0.97921432
        self.config['grasp_to_ee']['orientation']['x'] = -0.20277454
        self.config['grasp_to_ee']['orientation']['y'] = 0.00454233
        self.config['grasp_to_ee']['orientation']['z'] = -0.00107904

        self.graspToIiiwaLinkEE = spartanUtils.transformFromPose(self.config['grasp_to_ee'])
        self.iiwaLinkEEToGraspFrame = self.graspToIiiwaLinkEE.GetLinearInverse()

        pos = [-0.15, 0, 0]
        quat = [1,0,0,0]
        self.preGraspToGraspTransform = transformUtils.transformFromPose(pos, quat)

    def setupSubscribers(self):
        self.pointCloudSubscriber = rosUtils.SimpleSubscriber(self.pointCloudTopic, sensor_msgs.msg.PointCloud2)

        self.pointCloudSubscriber.start()

    def setupTF(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)


    def getDepthOpticalFrameToGraspFrameTransform(self):
        depthOpticalFrameToBase = self.tfBuffer.lookup_transform(self.graspFrameName, self.depthOpticalFrameName, rospy.Time(0))

        print depthOpticalFrameToBase
        return depthOpticalFrameToBase


    """
    Captures the current PointCloud2 from the sensor. Also records the pose of camera frame.
    """
    def capturePointCloudAndCameraTransform(self, cameraOrigin = [0,0,0]):
    	# sleep to transforms can update
    	rospy.sleep(0.5)
        msg = spartan_grasp_msgs.msg.PointCloudWithTransform()
        msg.header.stamp = rospy.Time.now()

        msg.camera_origin.x = cameraOrigin[0]
        msg.camera_origin.y = cameraOrigin[1]
        msg.camera_origin.z = cameraOrigin[2]

        msg.point_cloud_to_base_transform = self.getDepthOpticalFrameToGraspFrameTransform()

        msg.point_cloud = self.pointCloudSubscriber.waitForNextMessage()

        self.testData = msg # for debugging
        return msg

    def moveHome(self):
    	rospy.loginfo("moving home")
    	self.robotService.moveToJointPosition(self.storedPoses[self.config['home_pose_name']], maxJointDegreesPerSecond=self.config['scan']['joint_speed'])

    # scans to several positions
    def collectSensorData(self, saveToBagFile=False):

    	rospy.loginfo("collecting sensor data")

        pointCloudListMsg = spartan_grasp_msgs.msg.PointCloudList()
        pointCloudListMsg.header.stamp = rospy.Time.now()

        data = dict()

        for poseName in self.config['scan']['pose_list']:
            joint_positions = self.storedPoses[poseName]
            self.robotService.moveToJointPosition(joint_positions, maxJointDegreesPerSecond=self.config['scan']['joint_speed'])

            pointCloudWithTransformMsg = self.capturePointCloudAndCameraTransform()

            pointCloudListMsg.point_cloud_list.append(pointCloudWithTransformMsg)
            data[poseName] = pointCloudWithTransformMsg


        self.moveHome()
        self.sensorData = data
        self.pointCloudListMsg = pointCloudListMsg
        return pointCloudListMsg

    def requestGrasp(self, pointCloudListMsg):

    	rospy.loginfo("requesting grasp from spartan_grasp")

        serviceName = 'spartan_grasp/GenerateGraspsFromPointCloudList'
        rospy.wait_for_service(serviceName)
        s = rospy.ServiceProxy(serviceName, spartan_grasp_msgs.srv.GenerateGraspsFromPointCloudList)
        response = s(pointCloudListMsg)

        print "num scored_grasps = ", len(response.scored_grasps)
        if len(response.scored_grasps) == 0:
        	rospy.loginfo("no valid grasps found")
        	return False

        self.topGrasp = response.scored_grasps[0]
        rospy.loginfo("-------- top grasp score = %.3f", self.topGrasp.score)
        self.graspFrame = spartanUtils.transformFromROSPoseMsg(self.topGrasp.pose.pose)

    def getIiwaLinkEEFrameFromGraspFrame(self, graspFrame):
    	return transformUtils.concatenateTransforms([self.iiwaLinkEEToGraspFrame, graspFrame])

        # print "response ", response

    def moveToFrame(self, graspFrame):
    	iiwaLinkEEFrame = self.getIiwaLinkEEFrameFromGraspFrame(graspFrame)
    	poseDict = spartanUtils.poseFromTransform(iiwaLinkEEFrame)
    	poseMsg = rosUtils.ROSPoseMsgFromPose(poseDict)
    	poseStamped = geometry_msgs.msg.PoseStamped()
    	poseStamped.pose = poseMsg
    	poseStamped.header.frame_id = "base"

    	self.poseStamped = poseStamped
    	self.robotService.moveToCartesianPosition(poseStamped, self.config['grasp_speed'])

    def moveToGraspFrame(self, graspFrame):
    	preGraspFrame = transformUtils.concatenateTransforms([self.preGraspToGraspTransform, self.graspFrame])
    	vis.updateFrame(preGraspFrame, 'pre grasp frame', scale=0.15)
    	vis.updateFrame(graspFrame, 'grasp frame', scale=0.15)


    	self.moveToFrame(preGraspFrame)
    	# rospy.sleep(1.0)
    	self.moveToFrame(graspFrame)


    def testMoveToFrame(self):
    	pos = [ 0.51148583,  0.0152224 ,  0.50182436]
    	quat = [ 0.68751512,  0.15384615,  0.69882778, -0.12366916]
    	targetFrame = transformUtils.transformFromPose(pos, quat)
    	poseDict = spartanUtils.poseFromTransform(targetFrame)
    	poseMsg = rosUtils.ROSPoseMsgFromPose(poseDict)


    	poseStamped = geometry_msgs.msg.PoseStamped()
    	poseStamped.pose = poseMsg
    	poseStamped.header.frame_id = "base"
    	self.poseStamped = poseStamped

    	self.robotService.moveToCartesianPosition(poseStamped, 30)


    def showGraspFrame(self):
    	vis.updateFrame(self.graspFrame, 'grasp frame', scale=0.15)
    	vis.updateFrame(self.getIiwaLinkEEFrameFromGraspFrame(self.graspFrame), 'iiwa_link_ee_grasp_frame', scale=0.15)


    def saveSensorDataToBagFile(self, pointCloudListMsg=None, filename=None, overwrite=True):
        if pointCloudListMsg is None:
            pointCloudListMsg = self.pointCloudListMsg

        if filename is None:
            filename = os.path.join(spartanUtils.getSpartanSourceDir(), 'logs', 'grasp_sensor_data.bag')

        if overwrite and os.path.isfile(filename):
            os.remove(filename)

        bag = rosbag.Bag(filename, 'w')
        bag.write('data', pointCloudListMsg)
        bag.close()

    def testInThread(self):
        self.collectSensorData()
        self.requestGrasp(self.pointCloudListMsg)
        print "test finished"

    def testMoveHome(self):
   		self.taskRunner.callOnThread(self.moveHome)

    def test(self):
        self.taskRunner.callOnThread(self.testInThread)

    def testMoveToGrasp(self):
    	self.taskRunner.callOnThread(self.moveToGraspFrame, self.graspFrame)

   	


    @staticmethod
    def makeDefault():
        storedPosesFile = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects', 'station_config','RLG_iiwa_1','stored_poses.yaml')

        return GraspSupervisor(storedPosesFile=storedPosesFile)
