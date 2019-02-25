# ROS
import rospy
import actionlib

# ROS custom
import wsg_50_common.msg

# spartan
import spartan.utils.ros_utils as rosUtils


class SchunkDriver(object):
    def __init__(self, commandTopic="/wsg50_driver/wsg50/gripper_control", statusTopic="/wsg50_driver/wsg50/status",
                 statusSubscriberCallback=None):
        self.commandTopic = commandTopic
        self.statusTopic = statusTopic
        self.statusSubscriberCallback = statusSubscriberCallback

        self.initialize()

    def initialize(self):
        self.setupDefaultMessages()
        self.setupPublishers()
        self.setupSubscribers()
        self.setupActions()
        self._setup_config()
        self._last_result = None

    def _setup_config(self):
        self._config = dict()
        self._config['gripper_closed_width_when_empty'] = 0.0134
        self._config["gripper_open_width"] = 0.1
        self._config['tol'] = 0.0003

    def setupSubscribers(self):
        self.lastStatusMsg = None
        self.statusSubscriber = rosUtils.SimpleSubscriber(self.statusTopic, wsg_50_common.msg.Status)
        self.statusSubscriber.start(queue_size=1)

    def setupPublishers(self):
        self.streaming_pub = rospy.Publisher(self.commandTopic, wsg_50_common.msg.CommandGoal, queue_size=1);

    def setupActions(self):
        self.command_action_client = actionlib.SimpleActionClient(self.commandTopic, wsg_50_common.msg.CommandAction)

    def send_goal(self, goal, timeout=2.0):
        """

        :param goal: The CommandGoal for the action service
        :type goal: wsg_50_common.msg.CommandGoal
        :param timeout: The timout in seconds for use during this action client
        call
        :return: wsg_50_common.msg.CommandResult
        """

        self.command_action_client.send_goal(goal)
        self.command_action_client.wait_for_result(timeout=rospy.Duration.from_sec(timeout))
        result = self.command_action_client.get_result()
        self._last_result = result
        return result

    def stream_goal(self, goal_msg):
        self.command_action_client.send_goal(goal_msg)

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
        self.closeGripperGoal.command.force = 80
        self.closeGripperGoal.command.stop_on_block = False

    @property
    def gripper_status(self):
        return self.statusSubscriber.waitForNextMessage()

    def sendOpenGripperCommand(self):
        return self.send_goal(self.openGripperGoal)

    def sendCloseGripperCommand(self):
        return self.send_goal(self.closeGripperGoal)

    def sendGripperCommand(self, width, force=40., speed=0.1, stop_on_block=False, timeout=2.0, stream=False):
        goal = wsg_50_common.msg.CommandGoal()
        goal.command.command_id = wsg_50_common.msg.Command.MOVE
        goal.command.width = width
        goal.command.speed = speed
        goal.command.force = force
        goal.command.stop_on_block = stop_on_block
        if not stream:
            return self.send_goal(goal, timeout=timeout)
        else:
            return self.stream_goal(goal)

    def gripper_has_object(self):
        """
        Returns true if gripper has object in had. This is done by checking position
        :return: True if gripper has object
        """
        status = self.gripper_status
        width = status.width

        gripper_is_open = width > (self._config['gripper_open_width'] - self._config['tol'])


        gripper_fully_closed = width < (self._config['gripper_closed_width_when_empty'] + self._config['tol'])

        gripper_has_object = (not gripper_is_open) and (not gripper_fully_closed)
        return gripper_has_object

    def closeGripper(self):
        """
        Closes the gripper and checks whether or not their is an object in gripper
        :return: True if gripper has object
        """

        result = self.sendCloseGripperCommand()
        return self.gripper_has_object()

