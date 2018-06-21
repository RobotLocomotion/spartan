import unittest
import subprocess
import sys
import os
import numpy as np
import time
import socket

# ROS
import rospy
import sensor_msgs.msg
import rosgraph


# spartan
from spartan.utils.ros_utils import SimpleSubscriber
from spartan.utils.ros_utils import RobotService


class IiwaSimulationTest(unittest.TestCase):

    def setUp(self):
        self._kill_all_procman_processes()
        self._all_processes = []
        self._launch_procman_and_start_simulator()


    def tearDown(self):
        self._kill_all_processes()
        self._kill_all_procman_processes()
        pass

    def _kill_all_procman_processes(self):
        """
        Kills all processes related to procman
        :return:
        :rtype:
        """
        cmd = ["pkill",  "-f",  "procman"]
        process = subprocess.call(cmd)


    def _launch_procman_and_start_simulator(self):
        """
        Launches procman, starts the simulation
        :return:
        :rtype:
        """
        deputy = self._launch_process_and_test(["/usr/bin/env", "bot-procman-deputy", "--name", "localhost"])

        sheriff = self._launch_process_and_test(["/usr/bin/env", "bot-procman-sheriff",
                                           "--no-gui", "--on-script-complete", "exit",
                                           os.path.expandvars("${SPARTAN_SOURCE_DIR}/apps/iiwa/iiwa_hardware.pmd"),
                                           "6.start_drake_iiwa_sim"])

        sheriff.wait()
        print "Sheriff returned code %d" %(sheriff.returncode)
        assert sheriff.returncode == 0, "Sheriff returned code %d" %(sheriff.returncode)


        # print "sleeping for 3 seconds . . . "
        # time.sleep(3.0)
        # print "done sleeping"
        #
        # # Get the arm state to check sim is running
        # rospy.init_node("iiwa_sim_test", anonymous=True)
        # self._robotSubscriber = SimpleSubscriber("/joint_states", sensor_msgs.msg.JointState)
        # self._robotSubscriber.start()
        #
        # # wait for 5 seconds for robot movement service and /joint_states to come up
        # wait_time = 5
        # start_time = time.time()
        # while (time.time() - start_time) < wait_time:
        #     if self._robotSubscriber.hasNewMessage:
        #         break
        #     time.sleep(1)
        #
        # assert self._robotSubscriber.hasNewMessage, "Never received robot joint positions on /joint_states topic"


    def _launch_process_and_test(self, args):
        """

        Launch a process, assert that it has started up normally
        :param args: list of arguments, e.g. [executable, command line args, etc]
        :type args: list of strings
        :return:
        :rtype:
        """
        p = subprocess.Popen(args)
        if p.poll() is not None:
            # Process launched and terminated.

            process_name = ''.join(args)
            assert p.returncode==0, "Process %s returned with nonzero code %d" %(process_name, p.returncode)

        self._all_processes.append(p)
        return p

    def _kill_all_processes(self, additional_processes=None):
        """
        Kill all processes in self._all_processes
        Also kill any additional processes that may have been passed in
        :param additional_processes:
        :type additional_processes:
        :return:
        :rtype:
        """

        for p in self._all_processes:
            if p.poll() is None:
                p.kill()

        if additional_processes is None:
            additional_processes = []

        for p in additional_processes:
            if p.poll() is None:
                p.kill()

    def _start_ros_node_and_wait_for_sim(self):
        """
        Starts the ros node, waits for sim to startup by waiting to get a message
        on the /joint_states channel
        :return:
        :rtype:
        """

        print "sleeping for 3 seconds . . . "
        time.sleep(3.0)
        print "done sleeping"

        # Get the arm state to check sim is running
        rospy.init_node("iiwa_sim_test", anonymous=True)
        self._robotSubscriber = SimpleSubscriber("/joint_states", sensor_msgs.msg.JointState)
        self._robotSubscriber.start()

        # wait for 5 seconds for robot movement service and /joint_states to come up
        wait_time = 5
        start_time = time.time()
        while (time.time() - start_time) < wait_time:
            if self._robotSubscriber.hasNewMessage:
                break
            time.sleep(1)

        self.assertTrue(self._robotSubscriber.hasNewMessage, "Never received robot joint positions on /joint_states topic")



    def test_simulator_startup(self):
        """
        Launches simulator, makes sure that it can ping roscore
        :return:
        :rtype:
        """

        self._start_ros_node_and_wait_for_sim()




    def test_move_arm(self):
        """
        Move the arm to a given position
        :return:
        :rtype:
        """

        self._start_ros_node_and_wait_for_sim()

        # make robot service
        robotService = RobotService.makeKukaRobotService()

        targetPosition = [0.5] * 7
        success = robotService.moveToJointPosition(targetPosition, timeout=5)
        self.assertTrue(success, msg="RobotService MoveToJointPosition returned failure ")

        # check that we actually reached the target position
        lastRobotJointPositions = self._robotSubscriber.lastMsg.position
        reached_target_position = np.linalg.norm(np.array(targetPosition) - np.array(lastRobotJointPositions[0:7])) < 0.1

        if not reached_target_position:
            print "last robot joint position", lastRobotJointPositions

        self.assertTrue(reached_target_position, "Robot didn't reach target position")


if __name__ == '__main__':
    unittest.main()