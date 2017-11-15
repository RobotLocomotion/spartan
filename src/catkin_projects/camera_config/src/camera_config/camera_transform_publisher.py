# ROS
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_ros

# Spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils
import director.transformUtils as transformUtils


class CameraTransformPublisher:

	def __init__(self):
		self.cameraName = rospy.get_param('~camera_name')
		self.cameraInfoFilename = rospy.get_param('~camera_info_filename')
		rospy.loginfo("camera_info_filename  = %s", self.cameraInfoFilename)

		cameraInfoYamlDict = spartanUtils.getDictFromYamlFilename(self.cameraInfoFilename)
		self.cameraTypes = ['rgb','depth']
		self.parseCameraInfo(cameraInfoYamlDict)

		self.broadcaster = tf2_ros.TransformBroadcaster()
		self.staticTransformBroadcaster = tf2_ros.StaticTransformBroadcaster()

		rospy.loginfo("finished initializing the node")


	def parseCameraInfo(self, cameraInfoYamlDict):
		
		self.cameraInfo = dict()

		for cameraType in self.cameraTypes:
			d = cameraInfoYamlDict[cameraType]

			data = dict()
			data['raw_data'] = d
            			
			# the optical frame is defined as here http://www.ros.org/reps/rep-0103.html#id21 and follows
			# the opencv convention https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			# z forward, x right, y down
			opticalToLinkVtk = spartanUtils.transformFromPose(d['extrinsics']['transform_to_reference_link'])

			# body is the body frame of the camera as defined here http://www.ros.org/reps/rep-0103.html#id21
			# x forward, y left, z up
			bodyToLinkVtk = CameraTransformPublisher.transformOpticalFrameToBodyFrame(opticalToLinkVtk)
			bodyToLinkPoseDict = spartanUtils.poseFromTransform(bodyToLinkVtk)
			bodyToLink = rosUtils.ROSTransformMsgFromPose(bodyToLinkPoseDict)

			
			cameraToLinkStamped = geometry_msgs.msg.TransformStamped()
			cameraToLinkStamped.transform = bodyToLink
			cameraToLinkStamped.child_frame_id = self.cameraName + '_' + cameraType + "_frame"
			cameraToLinkStamped.header.frame_id = d['extrinsics']['reference_link_name']

			data['camera_to_link_transform_stamped'] = cameraToLinkStamped
	

			self.cameraInfo[cameraType] = data

			rospy.loginfo('finished parsing data for camera %s', cameraToLinkStamped.header.frame_id)

	def broadcastTransforms(self):
		for key, val in self.cameraInfo.iteritems():
			t = val['camera_to_link_transform_stamped']
			t.header.stamp = rospy.Time.now()
			# self.broadcaster.sendTransform(t)
			self.staticTransformBroadcaster.sendTransform(t)
			# print t
			# rospy.loginfo("broadcasting frame for camera %s", key)
			


	"""
    DEPRECATED
	Takes in a dict and converts it to a sensor_msgs.msgs.CameraInfo message
	"""
	@staticmethod
	def parseSingleCameraInfo(d):
		
		msg = sensor_msgs.msg.CameraInfo()
		msg.width = d['width']
		msg.height = d['height']
		msg.distortion_model = d['distortion_model']
		msg.D = d['D']
		msg.K = d['K']
		msg.R = d['R']
		msg.P = d['P']
		msg.binning_x = d['binning_x']
		msg.binning_y = d['binning_y']

		return msg

	@staticmethod
	def transformOpticalFrameToBodyFrame(opticalFrame):
	    rpy = [-90,0,-90]
	    opticalToBody = transformUtils.frameFromPositionAndRPY([0,0,0], rpy)
	    bodyFrame = transformUtils.concatenateTransforms([opticalToBody.GetLinearInverse(), opticalFrame])
	    return bodyFrame


