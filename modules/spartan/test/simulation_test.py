import unittest
import subprocess
import psutil
import sys
import os
import numpy as np
import time
import socket

# ROS
import rospy
import actionlib
import sensor_msgs.msg
import geometry_msgs.msg
import rosgraph
import tf2_ros


# spartan
from spartan.utils.ros_utils import JointStateSubscriber
from spartan.utils.ros_utils import RobotService
import spartan.utils.ros_utils as rosUtils
import spartan.utils.utils as spartan_utils

# spartan ROS
import robot_msgs.msg



def make_cartesian_trajectory_goal_world_frame():

    # (array([0.588497  , 0.00716426, 0.5159925 ]), array([ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ]))

    pos = [0.588497  , 0.00716426, 0.5159925]
    quat = [ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ]

    goal = robot_msgs.msg.CartesianTrajectoryGoal()
    traj = goal.trajectory

    # frame_id = "iiwa_link_ee"
    frame_id = "base"
    ee_frame_id = "iiwa_link_ee"
    
    xyz_knot = geometry_msgs.msg.PointStamped()
    xyz_knot.header.frame_id = frame_id
    xyz_knot.point.x = 0
    xyz_knot.point.y = 0
    xyz_knot.point.z = 0
    traj.xyz_points.append(xyz_knot)

    xyz_knot = geometry_msgs.msg.PointStamped()
    xyz_knot.header.frame_id = frame_id
    xyz_knot.point.x = pos[0]
    xyz_knot.point.y = pos[1]
    xyz_knot.point.z = pos[2]

    traj.xyz_points.append(xyz_knot)

    traj.ee_frame_id = ee_frame_id

    traj.time_from_start.append(rospy.Duration(0.0))
    traj.time_from_start.append(rospy.Duration(2.0))

    quat_msg = geometry_msgs.msg.Quaternion()
    quat_msg.w = quat[0]
    quat_msg.x = quat[1]
    quat_msg.y = quat[2]
    quat_msg.z = quat[3]

    traj.quaternions.append(quat_msg)

    return goal


def make_cartesian_gains_msg():
    msg = robot_msgs.msg.CartesianGain()

    kp_rot = 5
    msg.rotation.x = kp_rot
    msg.rotation.x = kp_rot
    msg.rotation.x = kp_rot

    kp_trans = 10
    msg.translation.x = kp_trans
    msg.translation.y = kp_trans
    msg.translation.z = kp_trans

    return msg

def make_force_guard_msg():
    msg = robot_msgs.msg.ForceGuard()
    external_force = robot_msgs.msg.ExternalForceGuard()

    body_frame = "iiwa_link_ee"
    expressed_in_frame = "iiwa_link_ee"
    force_vec = 20*np.array([-1,0,0])

    external_force.force.header.frame_id = expressed_in_frame
    external_force.body_frame = body_frame
    external_force.force.vector.x = force_vec[0]
    external_force.force.vector.y = force_vec[1]
    external_force.force.vector.z = force_vec[2]

    msg.external_force_guards.append(external_force)

    return msg

class IiwaSimulationTest(unittest.TestCase):

    def setUp(self):
        self.kuka_joint_names = spartan_utils.get_kuka_joint_names()
        self._all_processes = []
        self._terminate_all_processes()
        self._launch_procman_and_start_simulator()


    def tearDown(self):
        self._terminate_all_processes()
        pass

    def _terminate_all_processes(self):
        """
        Kills all processes related to procman
        :return:
        :rtype:
        """
        print "PRE KILLING TREE"
        os.system("pstree -ap")
        children = self._all_processes  #get_children_pids(os.getpid())
        for proc in children:
            proc.terminate()
            proc.wait()
        print "POST KILLING TREE:"
        os.system("pstree -ap")
        print "DONE WITH PRERUN CLEANUP"


    def _launch_procman_and_start_simulator(self):
        """
        Launches procman, starts the simulation
        :return:
        :rtype:
        """
        deputy = self._launch_process_and_test(["/usr/bin/env", "bot-procman-deputy", "--name", "localhost"])
        self._all_processes.append(deputy)
        sheriff = self._launch_process_and_test(["/usr/bin/env", "bot-procman-sheriff",
                                           "--no-gui", "--on-script-complete", "exit",
                                           os.path.expandvars("${SPARTAN_SOURCE_DIR}/apps/iiwa/iiwa_hardware.pmd"),
                                           "6.start_drake_iiwa_sim"])

        sheriff.wait()
        print "Sheriff returned code %d" % (sheriff.returncode)
        assert sheriff.returncode == 0, "Sheriff returned code %d" %(sheriff.returncode)


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

        return p

    def _start_ros_node_and_wait_for_sim(self):
        """
        Starts the ros node, waits for sim to startup by waiting to get a message
        on the /joint_states channel
        :return:
        :rtype:
        """

        print "sleeping for 5 seconds . . . "
        time.sleep(5.0)
        print "done sleeping"

        # Get the arm state to check sim is running
        rospy.init_node("iiwa_sim_test", anonymous=True)
        self._robotSubscriber = JointStateSubscriber("/joint_states")

        # wait for 5 seconds for robot movement service and /joint_states to come up
        # with more joints than just the gripper
        wait_time = 5
        start_time = time.time()
        while (time.time() - start_time) < wait_time:
            if len(self._robotSubscriber.joint_timestamps.keys()) > 2:
                break
            print "Rostopic list: ",
            os.system("rostopic list")
            time.sleep(1)

        self.assertTrue(len(self._robotSubscriber.joint_timestamps.keys()) > 2, "Never received full robot joint positions on /joint_states topic")


    def setupTF(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def get_ee_frame_pose(self):
        # check that desired position matches actual
        self.setupTF()
        ee_frame_name = "iiwa_link_ee"
        world_frame_name = "base"
        iiwa_link_ee_to_world = self.tfBuffer.lookup_transform(world_frame_name, ee_frame_name, rospy.Time(0), rospy.Duration(1))

        return iiwa_link_ee_to_world


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
        lastRobotJointPositions = self._robotSubscriber.get_position_vector_from_joint_names(self.kuka_joint_names)
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

    def test_cartesian_space_plan(self):
        """
        Test the cartesian space plan
        """

        self._start_ros_node_and_wait_for_sim()

        above_table_pre_grasp = [0.04486168762069299, 0.3256606458812486, -0.033502080520812445, -1.5769091802934694, 0.05899249087322813, 1.246379583616529, 0.38912999977004026]
        targetPosition = above_table_pre_grasp

        robotService = rosUtils.RobotService.makeKukaRobotService()
        success = robotService.moveToJointPosition(targetPosition, timeout=5)
        self.assertTrue(success, msg="RobotService MoveToJointPosition returned failure ")

        # check that we actually reached the target position
        lastRobotJointPositions = self._robotSubscriber.get_position_vector_from_joint_names(self.kuka_joint_names)
        reached_target_position = np.linalg.norm(np.array(targetPosition) - np.array(lastRobotJointPositions[0:7])) < 0.1

        # now call the cartesian space plan service
        client = actionlib.SimpleActionClient("plan_runner/CartesianTrajectory", robot_msgs.msg.CartesianTrajectoryAction)

        print "waiting for server"
        client.wait_for_server()
        print "connected to server"

        
        goal = make_cartesian_trajectory_goal_world_frame()

        goal.gains.append(make_cartesian_gains_msg())
        goal.force_guard.append(make_force_guard_msg())
        

        print "sending goal"
        client.send_goal(goal)

        rospy.loginfo("waiting for CartesianTrajectory action result")
        client.wait_for_result()
        result = client.get_result()

        rospy.sleep(3) # wait for controller to settle

        success = (result.status.status == robot_msgs.msg.PlanStatus.FINISHED_NORMALLY)
        print "result:", result

        self.assertTrue(success, msg = "PlanStatus was not FINISHED_NORMALLY")

        # check the position
        # check that desired position matches actual
        self.setupTF()
        iiwa_link_ee_to_world = self.get_ee_frame_pose()


        pos_actual = np.array(rosUtils.pointMsgToList(iiwa_link_ee_to_world.transform.translation))
        pos_desired = np.array(rosUtils.pointMsgToList(goal.trajectory.xyz_points[-1].point))

        quat_actual = np.array(rosUtils.quatMsgToList(iiwa_link_ee_to_world.transform.rotation))
        quat_desired = np.array(rosUtils.quatMsgToList(goal.trajectory.quaternions[0]))

        pos_tol = 0.01 # within 5 cm
        orientation_tol = 10 # within 5 degrees

        pos_error = np.linalg.norm(pos_actual - pos_desired)
        orientation_error_deg = 180/np.pi * spartan_utils.compute_angle_between_quaternions(quat_actual, quat_desired)

        print "pos_error:\n", pos_error
        print "orientation error:\n", orientation_error_deg

        self.assertTrue(pos_error < pos_tol, msg="position error was above tolerance")
        self.assertTrue(orientation_error_deg < orientation_tol, msg = "orientation error was above tolerance")


def dev():
    o = IiwaSimulationTest()
    # o.setUp()
    o.test_ik_service()
    # o.tearDown()


if __name__ == '__main__':
    # NOTE: You can't run all of these tests at once without
    # using something like pytest --forked to run each test
    # in its own process. This is because many of the tests
    # need to set up a ROS node, which can only be done once
    # in a process, ever, BUT must be done after roscore is
    # started. Since roscore is started and restarted
    # many times, this doesn't work when running this
    # file like 'python simulation_test.py'.
    unittest.main()
