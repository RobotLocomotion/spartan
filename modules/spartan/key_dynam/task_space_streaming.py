from __future__ import print_function


from enum import Enum

# ROS
import rospy
import std_srvs.srv

# spartan
import robot_msgs.msg

class TaskSpaceStreamingState(Enum):
    STOPPED = 1
    RUNNING = 2

class TaskSpaceStreaming(object):
    """
    Helper method for managing the task space streaming
    """

    def __init__(self):
        self._state = TaskSpaceStreamingState.STOPPED
        self.setup_publishers()
        self.setup_service_proxies()

        self.reset_cache()

    def setup_publishers(self):
        """
        Setup the publishers
        :return:
        :rtype:
        """
        self._task_space_streaming_pub = rospy.Publisher('plan_runner/task_space_streaming_setpoint',
                              robot_msgs.msg.CartesianGoalPoint, queue_size=1)

    def setup_service_proxies(self):
        """
        Setup the service proxies
        :return:
        :rtype:
        """
        rospy.wait_for_service('plan_runner/init_task_space_streaming')
        self._init_task_space_streaming_proxy = rospy.ServiceProxy('plan_runner/init_task_space_streaming',
        robot_msgs.srv.StartStreamingPlan)

        rospy.wait_for_service("plan_runner/stop_plan")
        self._stop_plan_service_proxy = sp = rospy.ServiceProxy('plan_runner/stop_plan',
            std_srvs.srv.Trigger)

    def start_streaming(self):
        """
        Start the streaming plan
        :return:
        """
        self.reset_cache()
        print("starting streaming plan")
        init_msg = robot_msgs.srv.StartStreamingPlanRequest()
        res = self._init_task_space_streaming_proxy(init_msg)
        print("start streaming response:\n", res)
        self._state = TaskSpaceStreamingState.RUNNING


    def publish(self, msg):
        self._task_space_streaming_pub.publish(msg)
        self._state_data['msgs'].append(msg)

    def stop_streaming(self):
        """
        Stop streaming plan
        :return:
        """
        print("stopping streaming plan")
        self._stop_plan_service_proxy(std_srvs.srv.TriggerRequest())
        self._state = TaskSpaceStreamingState.STOPPED

    def reset_cache(self):
        self._state_data = {'msgs': []}


    def get_last_published_message(self):
        """
        Returns the last message that was published
        :return:
        """

        msg = None
        try:
            msg = self._state_data['msgs'][-1]
        except (KeyError, IndexError):
            msg = None

        return msg

