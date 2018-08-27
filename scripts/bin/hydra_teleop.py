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
import tf2_ros
import tf


# spartan
from spartan.utils.ros_utils import SimpleSubscriber, JointStateSubscriber
from spartan.utils.ros_utils import RobotService
import spartan.utils.ros_utils as ros_utils
import spartan.utils.utils as spartan_utils
import spartan.utils.transformations as transformations
import robot_control.control_utils as control_utils
# spartan ROS
import robot_msgs.msg

import razer_hydra.msg



def make_cartesian_trajectory_goal_world_frame(pos, quat, duration):

    # (array([0.588497  , 0.00716426, 0.5159925 ]), array([ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ]))


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
    traj.time_from_start.append(rospy.Duration(duration))

    quat_msg = geometry_msgs.msg.Quaternion()
    quat_msg.w = quat[0]
    quat_msg.x = quat[1]
    quat_msg.y = quat[2]
    quat_msg.z = quat[3]

    traj.quaternions.append(quat_msg)
    traj.quaternions.append(quat_msg)

    return goal

def make_cartesian_trajectory_goal_gripper_frame(pos, quat, duration):
    # (array([0.588497  , 0.00716426, 0.5159925 ]), array([ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ]))

    # Barely touching tabletop

    goal = robot_msgs.msg.CartesianTrajectoryGoal()
    traj = goal.trajectory

    # frame_id = "iiwa_link_ee"
    frame_id = "iiwa_link_ee"
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
    traj.time_from_start.append(rospy.Duration(duration))

    quat_msg = geometry_msgs.msg.Quaternion()
    quat_msg.w = quat[0]
    quat_msg.x = quat[1]
    quat_msg.y = quat[2]
    quat_msg.z = quat[3]

    traj.quaternions.append(quat_msg)
    traj.quaternions.append(quat_msg)

    return goal

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
    
def make_move_goal(trans, quat, duration):
    goal = make_cartesian_trajectory_goal_world_frame(
        pos = trans,
        quat = quat,
        duration = duration)
    goal.gains.append(make_cartesian_gains_msg(20, 20.))
    goal.force_guard.append(make_force_guard_msg(15.))
    return goal


def tf_matrix_from_pose(pose):
    trans, quat = pose
    mat = transformations.quaternion_matrix(quat)
    mat[:3,3] = trans
    return mat

def get_relative_tf_between_poses(pose_1, pose_2):
    tf_1 = tf_matrix_from_pose(pose_1)
    tf_2 = tf_matrix_from_pose(pose_2)
    return np.linalg.inv(tf_1).dot(tf_2)

if __name__ == "__main__":
    rospy.init_node('sandbox')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    robotSubscriber = JointStateSubscriber("/joint_states")
    
    print("Waiting for full kuka state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    print("got full state")

    hydraSubscriber = SimpleSubscriber("/hydra_calib", razer_hydra.msg.Hydra)
    hydraSubscriber.start()
    print("Waiting for hydra startup...")
    hydraSubscriber.waitForNextMessage()
    print("Got hydra.")

    # EE CONTROL VERSION
    client = actionlib.SimpleActionClient("plan_runner/CartesianTrajectory", robot_msgs.msg.CartesianTrajectoryAction)
    print "waiting for EE action server"
    client.wait_for_server()
    print "connected to EE action server"

    # Control of robot works like this:
    #   When the user is pressing no buttons on the
    #   wand, nothing happens.
    #   When the user depresses the top front trigger of
    #   the wand (button index 0), the current wand position
    #   is memorized as the origin, and any movement of the
    #   wand from there is executed as incremental
    #   movements of the end effector of the robot.
    #   When the user releases the top front trigger, control
    #   is ceased and pressing the button again re-zeros the robot.

    paddle_index = 0
    enable_move_button_index = 0
    enable_move_button_last_state = False
    start_pose_wand = None
    start_tf_ee = None

    while (1):
        latest_hydra_msg = hydraSubscriber.waitForNextMessage()
        
        try:
            current_pose_ee = ros_utils.poseFromROSTransformMsg(
                tfBuffer.lookup_transform("base", "iiwa_link_ee", rospy.Time()).transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Troubling looking up tf...")
            rate.sleep()
            continue

        # Check if the move button is press
        hydra_status = latest_hydra_msg.paddles[paddle_index]
        if hydra_status.buttons[enable_move_button_index] is False:
            enable_move_button_last_state = False
        else:
            if enable_move_button_last_state is False:
                start_pose_wand = ros_utils.poseFromROSTransformMsg(hydra_status.transform)
                start_tf_wand = tf_matrix_from_pose(start_pose_wand)
                start_pose_ee = current_pose_ee
                start_tf_ee = tf_matrix_from_pose(start_pose_ee)
                enable_move_button_last_state = True
            # Difference in tf from initial TF
            current_pose_wand = ros_utils.poseFromROSTransformMsg(hydra_status.transform)
            hydra_tf = get_relative_tf_between_poses(start_pose_wand, current_pose_wand)
            print("Relative TF:", hydra_tf)
            # Target TF for the EE will be its start TF plus this offset
            slerp_amount = 0.2
            target_tf_ee = start_tf_ee.dot(hydra_tf)
            target_trans_ee = slerp_amount*target_tf_ee[:3, 3] + (1. - slerp_amount)*np.array(start_pose_ee[0])
            target_quat_ee = slerp_amount*transformations.quaternion_from_matrix(
                target_tf_ee[:3, :3]) + (1. - slerp_amount)*np.array(start_pose_ee[1])
            goal = make_move_goal(
                target_trans_ee,
                target_quat_ee,
                duration=0.5)
            client.send_goal(goal)
            rospy.loginfo("waiting for CartesianTrajectory action result")
            client.wait_for_result()
            result = client.get_result()
