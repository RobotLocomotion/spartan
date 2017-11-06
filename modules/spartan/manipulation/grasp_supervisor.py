# system
import os

# ROS
import rospy
import sensor_msgs.msg
import tf2_ros
import rosbag

# spartan ROS
import spartan_grasp_msgs.msg

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils


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
		self.config['scan']['joint_speed'] = 30

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
		msg = spartan_grasp_msgs.msg.PointCloudWithTransform()
		msg.header.stamp = rospy.Time.now()
		
		msg.camera_origin.x = cameraOrigin[0]
		msg.camera_origin.y = cameraOrigin[1]
		msg.camera_origin.z = cameraOrigin[2]

		msg.point_cloud_to_base_transform = self.getDepthOpticalFrameToGraspFrameTransform()
		
		msg.point_cloud = self.pointCloudSubscriber.waitForNextMessage()

		self.testData = msg # for debugging
		return msg

	# scans to several positions
	def collectSensorData(self, saveToBagFile=False):
		
		pointCloudListMsg = spartan_grasp_msgs.msg.PointCloudList()
		pointCloudListMsg.header.stamp = rospy.Time.now()

		data = dict()

		for poseName in self.config['scan']['pose_list']:
			joint_positions = self.storedPoses[poseName]
			self.robotService.moveToJointPosition(joint_positions, maxJointDegreesPerSecond=self.config['scan']['joint_speed'])

			pointCloudWithTransformMsg = self.capturePointCloudAndCameraTransform()

			pointCloudListMsg.point_cloud_list.append(pointCloudWithTransformMsg)
			data[poseName] = pointCloudWithTransformMsg

		self.sensorData = data
		self.pointCloudListMsg = pointCloudListMsg
		return pointCloudListMsg

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

	def test(self):
		self.taskRunner.callOnThread(self.collectSensorData)

	@staticmethod
	def makeDefault():
		storedPosesFile = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects', 'station_config','RLG_iiwa_1','stored_poses.yaml')

		return GraspSupervisor(storedPosesFile=storedPosesFile)
