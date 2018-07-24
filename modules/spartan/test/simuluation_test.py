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
import geometry_msgs.msg
import rosgraph
import tf2_ros


# spartan
from spartan.utils.ros_utils import SimpleSubscriber
from spartan.utils.ros_utils import RobotService
import spartan.utils.ros_utils as rosUtils



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


    def setupTF(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)


    def _test_simulator_startup(self):
        """
        Launches simulator, makes sure that it can ping roscore
        :return:
        :rtype:
        """

        self._start_ros_node_and_wait_for_sim()




    def _test_move_arm(self):
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

    def test_ik_service(self):
        """
        Test the ik service

        :return:
        :rtype:
        """
        self._start_ros_node_and_wait_for_sim()

        above_table_pre_grasp = [0.04486168762069299, 0.3256606458812486, -0.033502080520812445, -1.5769091802934694,
                                 0.05899249087322813, 1.246379583616529, 0.38912999977004026]

        poseStamped = geometry_msgs.msg.PoseStamped()

        pos = [0.51003723, 0.02411757, 0.30524811]
        quat = [0.68763689, 0.15390449, 0.69872774, -0.12348466]

        poseStamped.pose.position.x = pos[0]
        poseStamped.pose.position.y = pos[1]
        poseStamped.pose.position.z = pos[2]

        poseStamped.pose.orientation.w = quat[0]
        poseStamped.pose.orientation.x = quat[1]
        poseStamped.pose.orientation.y = quat[2]
        poseStamped.pose.orientation.z = quat[3]

        robotService = rosUtils.RobotService.makeKukaRobotService()
        response = robotService.runIK(poseStamped, seedPose=above_table_pre_grasp, nominalPose=above_table_pre_grasp)

        print "IK solution found ", response.success

        if response.success:
            print "moving to joint position", response.joint_state.position
            robotService.moveToJointPosition(response.joint_state.position)

        self.assertTrue(response.success)


        # check that desired position matches actual
        self.setupTF()
        ee_frame_name = "iiwa_link_ee"
        world_frame_name = "base"
        iiwa_link_ee_to_world = self.tfBuffer.lookup_transform(world_frame_name, ee_frame_name, rospy.Time(0), rospy.Duration(1))

        pos_actual_xyz = iiwa_link_ee_to_world.transform.translation
        pos_actual = [0]*3
        pos_actual[0] = pos_actual_xyz.x
        pos_actual[1] = pos_actual_xyz.y
        pos_actual[2] = pos_actual_xyz.z


        eps = 0.01
        pos_achieved = np.linalg.norm(np.array(pos) - np.array(pos_actual) ) < eps
        self.assertTrue(pos_achieved, "Didn't achieve desired end-effector position")


def dev():
    o = IiwaSimulationTest()
    # o.setUp()
    o.test_ik_service()
    # o.tearDown()


if __name__ == '__main__':
    unittest.main()

