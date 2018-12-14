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
import trajectory_msgs.msg
import rosgraph
import std_srvs.srv
import tf2_ros
import tf


# spartan
from spartan.utils.ros_utils import SimpleSubscriber, JointStateSubscriber
from spartan.utils.ros_utils import RobotService
import spartan.utils.ros_utils as ros_utils
import spartan.utils.utils as spartan_utils
import spartan.utils.transformations as transformations
import robot_control.control_utils as control_utils
from spartan.manipulation.schunk_driver import SchunkDriver
# spartan ROS
import robot_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf2_msgs.msg

import math


# Control of robot works like this:
#   When the user is pressing no buttons on the right
#   controller, nothing happens.
#   When the user presses on the trackpad of the right controller
#   (button 2), the current controller pose
#   is memorized as the origin. Any movement of the
#   controller from there is executed as incremental
#   movements of the end effector of the robot.
#   When the user releases the trackpad, control
#   is ceased and pressing the button again re-zeros the robot.

def make_cartesian_gains_msg(kp_rot, kp_trans):
    msg = robot_msgs.msg.CartesianGain()

    msg.rotation.x = kp_rot
    msg.rotation.y = kp_rot
    msg.rotation.z = kp_rot

    msg.translation.x = kp_trans
    msg.translation.y = kp_trans
    msg.translation.z = kp_trans

    return msg

def make_force_guard_msg(scale):
    msg = robot_msgs.msg.ForceGuard()
    external_force = robot_msgs.msg.ExternalForceGuard()

    body_frame = "iiwa_link_ee"
    expressed_in_frame = "iiwa_link_ee"
    force_vec = scale*np.array([-1,0,0])

    external_force.force.header.frame_id = expressed_in_frame
    external_force.body_frame = body_frame
    external_force.force.vector.x = force_vec[0]
    external_force.force.vector.y = force_vec[1]
    external_force.force.vector.z = force_vec[2]

    msg.external_force_guards.append(external_force)

    return msg

def ro(quat):
    return [quat[1], quat[2], quat[3], quat[0]]

def tf_matrix_from_pose(pose):
    trans, quat = pose
    mat = transformations.quaternion_matrix(quat)
    mat[:3, 3] = trans
    return mat

def do_main():
    rospy.init_node('sandbox', anonymous=True)

    # setup listener for tf2s (used for ee and controller poses)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    robotSubscriber = JointStateSubscriber("/joint_states")
    
    # wait until robot state found
    robotSubscriber = ros_utils.JointStateSubscriber("/joint_states")
    print("Waiting for full kuka state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    print("got full state")

    # init controller state (i.e. which buttons are pressed) is received
    rightControllerState = SimpleSubscriber("/vive_right", sensor_msgs.msg.Joy)
    rightControllerState.start()
    print("Waiting for vive...")
    rightControllerState.waitForNextMessage()
    print("Got vive controller state")

    # init button ids 
    move_button_id = 2 # press trackpad to move
    reset_button_id = 3 # press menu to reset
    trigger_axis_id = 0 # hold trigger to close gripper

    # init gripper
    handDriver = SchunkDriver()
    gripper_goal_pos = 0.1


    # Start by moving to an above-table pregrasp pose that we know
    # EE control will work well from (i.e. far from singularity)
    above_table_pre_grasp = [0.04486168762069299, 0.3256606458812486, -0.033502080520812445, -1.5769091802934694, 0.05899249087322813, 1.246379583616529, 0.38912999977004026]
    robotService = ros_utils.RobotService.makeKukaRobotService()
    success = robotService.moveToJointPosition(above_table_pre_grasp, timeout=5)
    print("Moved to position")

    # Then kick off task space streaming
    sp = rospy.ServiceProxy('plan_runner/init_task_space_streaming',
        robot_msgs.srv.StartStreamingPlan)
    init = robot_msgs.srv.StartStreamingPlanRequest()
    init.force_guard.append(make_force_guard_msg(20.))
    res = sp(init)
    
    # save plan number so we know when plan has ended
    plan_number = res.plan_number

    # set up client that checks plan number periodically
    client = actionlib.SimpleActionClient("plan_runner/GetPlanNumber", robot_msgs.msg.GetPlanNumberAction)
    client.wait_for_server()


    print("Started task space streaming")
    pub = rospy.Publisher('plan_runner/task_space_streaming_setpoint',
        robot_msgs.msg.CartesianGoalPoint, queue_size=1);


    def cleanup():
        rospy.wait_for_service("plan_runner/stop_plan")
        sp = rospy.ServiceProxy('plan_runner/stop_plan',
            std_srvs.srv.Trigger)
        init = std_srvs.srv.TriggerRequest()
        print sp(init)
        print("Done cleaning up and stopping streaming plan")

    frame_name = "iiwa_link_ee" # end effector frame name

    rospy.on_shutdown(cleanup)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100) # max rate at which control should happen

    illegal_move = False

    try:
        # variables to track when we lasted updated gripper and if we are moving arm
        last_gripper_update_time = time.time()
        last_move_pressed = False

        # control loop
        while not rospy.is_shutdown():

            # Check if plan ended
            goal = robot_msgs.msg.GetPlanNumberGoal()
            client.send_goal(goal)   
            client.wait_for_result()
            result = client.get_result()

            if (result.plan_number != plan_number):
                print "Illegal move, restarting plan"
                illegal_move = True
                sp = rospy.ServiceProxy('plan_runner/init_task_space_streaming',
                    robot_msgs.srv.StartStreamingPlan)
                init = robot_msgs.srv.StartStreamingPlanRequest()
                init.force_guard.append(make_force_guard_msg(20.))
                res = sp(init)
                
                plan_number = res.plan_number                


            # get controller button / trigger data
            latest_state_msg = rightControllerState.last_message

            # Check if move button pressed
            move_pressed = (latest_state_msg.buttons[move_button_id] == 1)
            reset_pressed = (latest_state_msg.buttons[reset_button_id] == 1)

            if reset_pressed and not move_pressed:
                cleanup()
                do_main()
                return


            # Gripper 
            dt = time.time() - last_gripper_update_time
            if dt > 0.2:
                last_gripper_update_time = time.time()
                gripper_goal_pos = 0.1 * (1-latest_state_msg.axes[trigger_axis_id])
                handDriver.sendGripperCommand(gripper_goal_pos, speed=0.1, timeout=0.01)
                print "Gripper goal pos: ", gripper_goal_pos


            # get current tf from ros world to ee
            try:
                ee_pose_current = ros_utils.poseFromROSTransformMsg(
                    tfBuffer.lookup_transform("base", frame_name, rospy.Time()).transform)
                ee_tf_current = tf_matrix_from_pose(ee_pose_current)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Troubling looking up robot pose...")
                rate.sleep()
                continue

            # get current tf from vr world to controller
            try:
                pose_current_vr_controller = ros_utils.poseFromROSTransformMsg(
                    tfBuffer.lookup_transform("world", "right_controller", rospy.Time()).transform)
                tf_current_vr_controller = tf_matrix_from_pose(pose_current_vr_controller)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Troubling looking up controller pose...")
                rate.sleep()
                continue

            # get tf from ros world to vr world (should be constant while moving)
            try:
                pose_ros_vr = ros_utils.poseFromROSTransformMsg(
                    tfBuffer.lookup_transform("base", "world", rospy.Time()).transform)
                tf_ros_vr = tf_matrix_from_pose(pose_ros_vr)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Troubling looking up tf from vr world to ros world...")
                rate.sleep()
                continue

            controller_tf_current = tf_ros_vr.dot(tf_current_vr_controller);


            # Releasing move button clears illegal move
            if not move_pressed and illegal_move:
                illegal_move = False
            
            if move_pressed and not illegal_move:
                if last_move_pressed is False:
                    # save starting controller and ee poses
                    ee_tf_start = ee_tf_current
                    controller_tf_start = controller_tf_current

                # calculate rotation from controller start pose to current and apply to ee
                controller_tf_start_inv = np.linalg.inv(controller_tf_start)
                controller_tf_start_to_current = controller_tf_current.dot(controller_tf_start_inv)
                ee_tf_target = controller_tf_start_to_current.dot(ee_tf_start)

                # extract and normalize quat from tf
                target_quat_ee = transformations.quaternion_from_matrix(ee_tf_target)
                target_quat_ee = np.array(target_quat_ee) / np.linalg.norm(target_quat_ee)

                # calculate controller position delta and add to start position to get target ee position
                controller_trans = controller_tf_current[:3, 3] - controller_tf_start[:3, 3]
                target_trans_ee = ee_tf_start[:3, 3] + controller_trans
                
                # publish target pose as cartesian goal point
                new_msg = robot_msgs.msg.CartesianGoalPoint()
                new_msg.xyz_point.header.frame_id = "world"
                new_msg.xyz_point.point.x = target_trans_ee[0]
                new_msg.xyz_point.point.y = target_trans_ee[1]
                new_msg.xyz_point.point.z = target_trans_ee[2]
                new_msg.xyz_d_point.x = 0.
                new_msg.xyz_d_point.y = 0.
                new_msg.xyz_d_point.z = 0.0
                new_msg.quaternion.w = target_quat_ee[0]
                new_msg.quaternion.x = target_quat_ee[1]
                new_msg.quaternion.y = target_quat_ee[2]
                new_msg.quaternion.z = target_quat_ee[3]
                new_msg.gain = make_cartesian_gains_msg(5., 10.)
                new_msg.ee_frame_id = frame_name
                pub.publish(new_msg)
                print(new_msg)


            last_move_pressed = move_pressed
            rate.sleep()



    except Exception as e:
        print "Suffered exception ", e

if __name__ == "__main__":
    do_main()
