# ROS
import rospy

# ROS custom
import wsg50_msgs.msg

# spartan
import spartan.utils.ros_utils as rosUtils




class SchunkDriver(object):

	def __init__(self, commandTopic="/schunk_driver/schunk_wsg_command", statusTopic="/schunk_driver/schunk_wsg_status",
				 statusSubscriberCallback=None):
		self.commandTopic = commandTopic
		self.statusTopic = statusTopic
		self.initialize(statusSubscriberCallback)

	def initialize(self, statusSubscriberCallback):
		self.setupDefaultMessages()
		if statusSubscriberCallback is not None:
			self.setupSubscribers(statusSubscriberCallback)
		self.setupPublishers()

	def setupSubscribers(self, statusSubscriberCallback):
		self.statusSubscriber = rosUtils.SimpleSubscriber(self.statusTopic, wsg50_msgs.msg.WSG_50_state, statusSubscriberCallback)
		self.statusSubscriber.start(queue_size=1)

	def setupPublishers(self):
		self.commandPublisher = rospy.Publisher(self.commandTopic, wsg50_msgs.msg.WSG_50_command, queue_size=1)


	def setupDefaultMessages(self):
		self.openGripperMsg = wsg50_msgs.msg.WSG_50_command()
		self.openGripperMsg.position_mm = 100
		self.openGripperMsg.force = 40

		self.closeGripperMsg = wsg50_msgs.msg.WSG_50_command()
		self.closeGripperMsg.position_mm = 0
		self.closeGripperMsg.force = 40

	def sendOpenGripperCommand(self):
		self.openGripperMsg.header.stamp = rospy.Time.now()
		self.commandPublisher.publish(self.openGripperMsg)


	def sendCloseGripperCommand(self):
		self.closeGripperMsg.header.stamp = rospy.Time.now()
		self.commandPublisher.publish(self.closeGripperMsg)


	def sendGripperCommand(self, position, force):
		msg = wsg50_msgs.msg.WSG_50_command()
		msg.header.stamp = rospy.Time.now()
		msg.position_mm = position
		msg.force = force
		self.commandPublisher.publish(msg)

	"""
	Closes the gripper and checks whether or not their is an object in gripper
	"""
	def closeGripper(self, sleepTime=0.5, gripperPositionWhenEmpty=0.5):
		self.sendCloseGripperCommand()
		rospy.sleep(sleepTime) # wait for gripper to close
		objectInGripper = True
		return objectInGripper


