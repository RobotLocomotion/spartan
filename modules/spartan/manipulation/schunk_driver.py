# ROS
import rospy
import actionlib

# ROS custom
import wsg_50_common.msg

# spartan
import spartan.utils.ros_utils as rosUtils




class SchunkDriver(object):

	def __init__(self, commandTopic="/wsg50_driver/wsg50/gripper_control", statusTopic="/wsg50_driver/wsg50/status", statusSubscriberCallback=None):
		self.commandTopic = commandTopic
		self.statusTopic = statusTopic
		self.statusSubscriberCallback = statusSubscriberCallback
		self.initialize()

	def initialize(self):
		self.setupDefaultMessages()
		self.setupPublishers()
		self.setupSubscribers()
		self.setupActions()
		self._last_result = None

	def setupSubscribers(self):
		self.lastStatusMsg = None
		self.statusSubscriber = rosUtils.SimpleSubscriber(self.statusTopic, wsg_50_common.msg.Status)
		self.statusSubscriber.start(queue_size=1)

	def setupPublishers(self):
		pass

	def setupActions(self):
		self.command_action_client = actionlib.SimpleActionClient(self.commandTopic, wsg_50_common.msg.CommandAction)

	def send_goal(self, goal, timeout=2.0):
		"""
		Sends goal via the action client
		"""
		self.command_action_client.send_goal(goal)
		self.command_action_client.wait_for_result(timeout=rospy.Duration.from_sec(2.0))
		result = self.command_action_client.get_result()
		self._last_result = result
		return result


	def setupDefaultMessages(self):
		# TODO(gizatt): Actually use the actionlib interface
		# rather than brute-forcing this.
		self.openGripperGoal = wsg_50_common.msg.CommandGoal()
		self.openGripperGoal.command.command_id = wsg_50_common.msg.Command.MOVE
		self.openGripperGoal.command.width = 0.1
		self.openGripperGoal.command.speed = 0.1
		self.openGripperGoal.command.force = 40
		self.openGripperGoal.command.stop_on_block = False

		self.closeGripperGoal = wsg_50_common.msg.CommandGoal()
		self.closeGripperGoal.command.command_id = wsg_50_common.msg.Command.MOVE
		self.closeGripperGoal.command.width = 0.0
		self.closeGripperGoal.command.speed = 0.1
		self.closeGripperGoal.command.force = 40
		self.closeGripperGoal.command.stop_on_block = False

	@property
	def gripper_status(self):
		return self.statusSubscriber.last_message()	

	def sendOpenGripperCommand(self):
		return self.send_goal(self.openGripperGoal)

	def sendCloseGripperCommand(self):
		return self.send_goal(self.closeGripperGoal)

	def sendGripperCommand(self, width, force=40., speed=0.1, stop_on_block=False):
		goal = wsg_50_common.msg.CommandGoal()
		goal.command.command_id = wsg_50_common.msg.Command.MOVE
		goal.command.width = position
		goal.command.speed = speed
		goal.command.force = force
		goal.command.stop_on_block = stop_on_block
		return self.send_goal(goal)

	
	def closeGripper(self, gripperPositionWhenEmpty=0.5):
		"""
		Closes the gripper and checks whether or not their is an object in gripper
		"""
		result = self.sendCloseGripperCommand()
		objectInGripper = True
		return objectInGripper


