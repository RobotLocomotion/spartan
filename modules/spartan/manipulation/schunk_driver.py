# ROS
import rospy

# ROS custom
import wsg_50_common.msg

# spartan
import spartan.utils.ros_utils as rosUtils




class SchunkDriver(object):

	def __init__(self, commandTopic="/wsg50_driver/wsg50/gripper_control/goal", statusTopic="/wsg50_driver/wsg50/status", statusSubscriberCallback=None):
		self.commandTopic = commandTopic
		self.statusTopic = statusTopic
		self.statusSubscriberCallback = statusSubscriberCallback
		self.initialize()

	def initialize(self):
		self.setupDefaultMessages()
		self.setupPublishers()
		self.setupSubscribers()

	def setupSubscribers(self):
		self.lastStatusMsg = None
		self.statusSubscriber = rosUtils.SimpleSubscriber(self.statusTopic, wsg_50_common.msg.Status, self.handleStatusCallback)
		self.statusSubscriber.start(queue_size=1)

	def setupPublishers(self):
		self.commandPublisher = rospy.Publisher(self.commandTopic, wsg_50_common.msg.CommandActionGoal, queue_size=1)

	def setupDefaultMessages(self):
		# TODO(gizatt): Actually use the actionlib interface
		# rather than brute-forcing this.
		self.openGripperMsg = wsg_50_common.msg.CommandActionGoal()
		self.openGripperMsg.goal.command.command_id = wsg_50_common.msg.Command.MOVE
		self.openGripperMsg.goal.command.width = 0.1
		self.openGripperMsg.goal.command.speed = 0.1
		self.openGripperMsg.goal.command.force = 40
		self.openGripperMsg.goal.command.stop_on_block = False

		self.closeGripperMsg = wsg_50_common.msg.CommandActionGoal()
		self.closeGripperMsg.goal.command.command_id = wsg_50_common.msg.Command.MOVE
		self.closeGripperMsg.goal.command.width = 0.0
		self.closeGripperMsg.goal.command.speed = 0.1
		self.closeGripperMsg.goal.command.force = 40
		self.closeGripperMsg.goal.command.stop_on_block = False

	def handleStatusCallback(self, msg):
		self.lastStatusMsg = msg
		# If the width, speed, and force are all *exactly*
		# zero, the gripper hasn't been home'd, and won't work
		# until it is homed.
		# TODO(gizatt): Can we detected home'd-ness without
		# relying on this hack?
		if (self.lastStatusMsg.width == 0.0 and
		    self.lastStatusMsg.current_speed == 0.0 and
		    self.lastStatusMsg.current_force == 0.0):
			self.reset_and_home()
		if self.statusSubscriberCallback is not None:
			self.statusSubscriberCallback(msg)

	def sendOpenGripperCommand(self):
		self.openGripperMsg.header.stamp = rospy.Time.now()
		self.commandPublisher.publish(self.openGripperMsg)

	def sendCloseGripperCommand(self):
		self.closeGripperMsg.header.stamp = rospy.Time.now()
		self.commandPublisher.publish(self.closeGripperMsg)

	def reset_and_home(self):
		resetMsg = wsg_50_common.msg.CommandActionGoal()
		resetMsg.goal.command.command_id = wsg_50_common.msg.Command.ACKNOWLEDGE_ERROR
		self.commandPublisher.publish(resetMsg)
		homeMsg = wsg_50_common.msg.CommandActionGoal()
		homeMsg.goal.command.command_id = wsg_50_common.msg.Command.HOMING
		self.commandPublisher.publish(homeMsg)

	def sendGripperCommand(self, position, force=40., speed=0.1, stop_on_block=False):
		msg = wsg_50_common.msg.CommandActionGoal()
		msg.header.stamp = rospy.Time.now()
		msg.goal.command.command_id = wsg_50_common.msg.Command.MOVE
		msg.goal.command.width = position
		msg.goal.command.speed = speed
		msg.goal.command.force = force
		msg.goal.command.stop_on_block = stop_on_block
		self.commandPublisher.publish(msg)

	"""
	Closes the gripper and checks whether or not their is an object in gripper
	"""
	def closeGripper(self, sleepTime=1.5, gripperPositionWhenEmpty=0.5):
		self.sendCloseGripperCommand()
		rospy.sleep(sleepTime) # wait for gripper to close
		objectInGripper = True
		return objectInGripper
