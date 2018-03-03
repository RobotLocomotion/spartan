#!/usr/bin/env python
import subprocess
import sys
import os
import sensor_msgs.msg


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
        "4.sim_pybullet_startup_for_ci"])

    sheriff.wait()
    if sheriff.returncode != 0:
        print "Sheriff returned code ", str(sheriff.returncode)
        cleanup_and_exit(1)


def try_to_move_arm_and_hand():
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
    poseStamped = geometry_msgs.msg.PoseStamped()
    poseStamped.pose.position.x = 0.5
    poseStamped.pose.position.y = 0.0
    poseStamped.pose.position.z = 1.5
    poseStamped.pose.orientation.w = 1.0
    poseStamped.pose.orientation.x = 0.0
    poseStamped.pose.orientation.y = 0.0
    poseStamped.pose.orientation.z = 0.0
    srvRequest = robot_msgs.srv.RunIKRequest()
    srvRequest.pose_stamped = poseStamped
    success = robotService.moveToCartesianPosition(srvRequest, timeout=5)
    if not success:
        print "robotService moveToCartesianPosition returned failure ", success
        cleanup_and_exit(1)


    # Close the gripper
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