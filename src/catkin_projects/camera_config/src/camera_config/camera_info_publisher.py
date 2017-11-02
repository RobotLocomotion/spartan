import rospy


class CameraInfoPublisher:

	def __init__(self):
		self.cameraName = '/camera'
		self.cameraInfoFilename = rospy.get_param('~camera_info_filename')

		rospy.loginfo("camera_info_filename  = %s", self.cameraInfoFilename)
		rospy.loginfo("finished initializing the node")
