#!/usr/bin/env python
import subprocess
import sys
import os
import sensor_msgs.msg
import spartan.utils.ros_utils as rosUtils

import tf2_ros

all_processes = []

def launch_process_and_test(args):
    p = subprocess.Popen(args)
    if p.poll() is not None:
        # Process launched and terminated.
        if p.returncode != 0:
            print "Process \"", " ".join(args), "\" returned with nonzero code ", str(p.returncode)
            sys.exit(1)

    all_processes.append(p)
    return p

def kill_all_processes():
    for p in all_processes:
        if p.poll() is None:
            p.kill()

def cleanup_and_exit(error_code):
    kill_all_processes()
    sys.exit(error_code)

def try_launch_deputy_and_sheriff():
    # Create a procman deputy
    deputy = launch_process_and_test(["/usr/bin/env", "bot-procman-deputy", "--name", "localhost"])

    # Run the procman sheriff in script mode to launch a full simulator
    sheriff = launch_process_and_test(["/usr/bin/env", "bot-procman-sheriff", 
        "--no-gui", "--on-script-complete", "exit", 
        os.path.expandvars("${SPARTAN_SOURCE_DIR}/apps/iiwa/iiwa_hardware.pmd"),
        "6.start_drake_iiwa_sim"])

    sheriff.wait()
    if sheriff.returncode != 0:
        print "Sheriff returned code ", str(sheriff.returncode)
        cleanup_and_exit(1)


def try_to_move_arm_and_hand(move_hand=False):
    import rospy
    import rosgraph
    import socket
    import numpy as np
    import time

    try:
        rosgraph.Master('/rostopic').getPid()
    except socket.error:
        print "Unable to communicate with ROS master, even though it should be up!"
        cleanup_and_exit(1)
    rospy.init_node("test_full_simulation_stack", anonymous=True)

    # Get the arm state
    from spartan.utils.ros_utils import SimpleSubscriber
    
    robotSubscriber = SimpleSubscriber("/joint_states", sensor_msgs.msg.JointState)
    robotSubscriber.start()

    # Move the arm
    from spartan.utils.ros_utils import RobotService
    robotService = RobotService.makeKukaRobotService()
    targetPosition = [0.5]*7
    success = robotService.moveToJointPosition(targetPosition, timeout=5)
    if not success:
        print "RobotService MoveToJointPosition returned failure ", success

    rospy.sleep(0.1)
    if robotSubscriber.hasNewMessage is False:
        print "We never received robot joint positions... what gives."
        cleanup_and_exit(1)

    lastRobotJointPositions = robotSubscriber.lastMsg.position
    if np.linalg.norm(np.array(targetPosition) - np.array(lastRobotJointPositions[0:7])) > 0.1:
        print "Did not reach target position: ", targetPosition, " vs current pos ", lastRobotJointPositions
        cleanup_and_exit(1)

    # Test IK utilities, which go into pydrake
    import geometry_msgs
    import robot_msgs.srv
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



    # check that desired position matches actual

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    ee_frame_name = "iiwa_link_ee"
    world_frame_name = "base"
    iiwa_link_ee_to_world = tfBuffer.lookup_transform(world_frame_name, ee_frame_name, rospy.Time(0),
                                                           rospy.Duration(1))

    pos_actual_xyz = iiwa_link_ee_to_world.transform.translation
    pos_actual = [0] * 3
    pos_actual[0] = pos_actual_xyz.x
    pos_actual[1] = pos_actual_xyz.y
    pos_actual[2] = pos_actual_xyz.z

    eps = 0.01
    pos_achieved = np.linalg.norm(np.array(pos) - np.array(pos_actual)) < eps

    if not response.success:
        print "robotService moveToCartesianPosition returned failure ", success
        cleanup_and_exit(1)

    if not pos_achieved:
        print "Didn't achieve desired end-effector position"
        cleanup_and_exit(1)


    # Close the gripper
    if move_hand:
        from spartan.manipulation.schunk_driver import SchunkDriver
        schunkDriver = SchunkDriver()

        rospy.sleep(1.0)

        if schunkDriver.lastStatusMsg is None:
            print "Got no starting gripper position. Is sim running?"
            cleanup_and_exit(1)
        startingGripperPosition = schunkDriver.lastStatusMsg.position_mm
        
        schunkDriver.sendCloseGripperCommand()
        
        rospy.sleep(1.0)

        if schunkDriver.lastStatusMsg.position_mm >= 10.0:
            print "After trying to close gripper, gripper position was %f (started at %f)." % (schunkDriver.lastStatusMsg.position_mm, startingGripperPosition)
            print "sendCloseGripperCommand appears ineffectual"
            cleanup_and_exit(1)


if __name__=="__main__":
    try:
        try_launch_deputy_and_sheriff()
    except Exception as e:
        print "Exception in test_full_simulation_stack.py when setting up deputy and sheriff: ", e
        cleanup_and_exit(1)

    try:
        # We have an environment! Now execute a canned
        # script to move the arm and close the gripper.
        try_to_move_arm_and_hand()
    except Exception as e:
        print "Exception in test_full_simulation_stack.py when running canned move script: ", e
        cleanup_and_exit(1)

    try:
        cleanup_and_exit(0)
    except Exception as e:
        print "Exception in test_full_simulation_stack.py when cleaning up: ", e
        cleanup_and_exit(1)