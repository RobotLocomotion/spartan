import rospy


class CameraInfoPublisher:

	def __init__(self):
		self.cameraName = '/camera'
		self.cameraInfoFilename = rospy.get_param(~camera_info_filename)

		rospy.loginfo("camera_info_filename  = %s", camera_info_filename)
		rospy.loginfo("finished initializing the node")



if __name__=="__main__":

	rospy.init_node("camera_info_publisher")

	rospy.spin