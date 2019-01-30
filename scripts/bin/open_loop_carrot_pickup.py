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


# spartan
from spartan.utils.ros_utils import JointStateSubscriber
from spartan.utils.ros_utils import RobotService
import spartan.utils.ros_utils as rosUtils
from spartan.manipulation.schunk_driver import SchunkDriver
import spartan.utils.utils as spartan_utils
import spartan.utils.transformations as transformations
import robot_control.control_utils as control_utils
# spartan ROS
import robot_msgs.msg
import carrot_msgs.msg



def make_cartesian_trajectory_goal(pos, quat, duration, frame_id, ee_frame_id):
    goal = robot_msgs.msg.CartesianTrajectoryGoal()
    traj = goal.trajectory
    
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

def make_downward_force_guard_msg(scale):
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

    
def make_world_move_goal(xyz, duration=5.0, max_force=15.):
    # TODO: yaw / rotations
    if (xyz[0] > 0.45 and xyz[0] < 0.8 and 
        xyz[1] > -0.3 and xyz[1] < 0.3 and
        xyz[2] > -0.05 and xyz[2] < 0.8):
        quat = np.array([0.688, 0.155, 0.692, -0.153])
        goal = make_cartesian_trajectory_goal(
            pos = xyz,
            quat = quat,
            duration = duration,
            frame_id="base",
            ee_frame_id="iiwa_link_ee")
        goal.gains.append(make_cartesian_gains_msg(20., 20.))
        goal.force_guard.append(make_downward_force_guard_msg(max_force))
        return goal
    else:
        rospy.logwarn("Out out bounds xyz: %f, %f, %f" % (xyz[0], xyz[1], xyz[2]))
        return None

def make_relative_gripper_movement_goal(xyz, quat, duration=5.0):
    goal = make_cartesian_trajectory_goal(
        pos = xyz,
        quat = quat,
        duration = duration,
        frame_id="iiwa_link_ee",
        ee_frame_id="iiwa_link_ee")
    # TODO: deal with rotations not being right
    goal.gains.append(make_cartesian_gains_msg(0., 20.))
    goal.force_guard.append(make_downward_force_guard_msg(25.))
    return goal


# Useful scanning positions
central_view_pose = np.array([4.9, 13.7, -1.0, -95.5, -3.7, 51.4, 29.3])*np.pi/180.

if __name__ == "__main__":
    rospy.init_node('carrot_collector', anonymous=True)
    
    robotSubscriber = JointStateSubscriber("/joint_states")
    configBundleSubscriber = rosUtils.SimpleSubscriber(
        "/carrot_configs", carrot_msgs.msg.CarrotConfigurationBundle)
    configBundleSubscriber.start()
    rospy.loginfo("Set up subscribers...")
    rospy.sleep(1.0)
    
    # EE CONTROL VERSION
    client = actionlib.SimpleActionClient("plan_runner/CartesianTrajectory", robot_msgs.msg.CartesianTrajectoryAction)
    rospy.loginfo("Waiting for action servers...")
    client.wait_for_server()

    rospy.loginfo("Waiting for hearing full robot state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)

    rospy.loginfo("Making robot server...")
    robotService = rosUtils.RobotService.makeKukaRobotService()

    schunkDriver = SchunkDriver()

    while (1):
        rospy.loginfo("Moving to start position.")
        schunkDriver.sendOpenGripperCommand()
        success = robotService.moveToJointPosition(central_view_pose, timeout=5)

        rospy.loginfo("Waiting for bundle.")
        config_bundle = configBundleSubscriber.waitForNextMessage()

        if rospy.is_shutdown():
            break

        rospy.loginfo("\nPROCESSING BUNDLE\n****************")
        for config in config_bundle.configurations:
            schunkDriver.sendOpenGripperCommand()

            pos, quat = ros_utils.poseFromROSTransformMsg(config.pose)
            height = config.height
            radius = config.radius

            goal = make_world_move_goal(np.array(pos) + np.array([0., 0., 0.3]), duration=3.0, max_force=15.)
            if goal is None:
                continue
            client.send_goal(goal)
            rospy.loginfo("Moving over carrot...")
            client.wait_for_result()
            result = client.get_result()

            goal = make_world_move_goal(np.array(pos) + np.array([0., 0., 0.1]), duration=4.0, max_force=15.)
            if goal is None:
                continue
            client.send_goal(goal)
            rospy.loginfo("Moving down to carrot...")
            client.wait_for_result()
            result = client.get_result()

            # That should force-block against the table. Back off a tiny bit vertically.
            goal = make_relative_gripper_movement_goal(
                np.array([0., 0., -0.005]),
                np.array([1., 0., 0., 0.]), duration=0.5)
            if goal is None:
                continue
            client.send_goal(goal)
            rospy.loginfo("Backing off a bit...")
            client.wait_for_result()
            result = client.get_result()

            rospy.sleep(0.5)
            has_object = schunkDriver.closeGripper()

            if has_object:
                rospy.loginfo("Got an object, picking to the bowl.")
                goal = make_world_move_goal(np.array(pos) + np.array([0., 0., 0.3]), duration=1.0, max_force=50.)
                if goal is None:
                    continue
                client.send_goal(goal)
                rospy.loginfo("Moving up...")
                client.wait_for_result()
                result = client.get_result()

                bowl_pos = [0.57, -0.2, 0.0]
                goal = make_world_move_goal(np.array(bowl_pos)+ np.array([0., 0., 0.25]), duration=1.0, max_force=15.)
                if goal is None:
                    continue
                client.send_goal(goal)
                rospy.loginfo("Moving over bowl...")
                client.wait_for_result()
                result = client.get_result()

                rospy.loginfo("Dropping.")
                schunkDriver.sendOpenGripperCommand()
                rospy.sleep(0.5)


            else:
                rospy.loginfo("Didn't get an object.")