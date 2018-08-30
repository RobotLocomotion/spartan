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
import spartan.utils.utils as spartan_utils
import robot_control.control_utils as control_utils
# spartan ROS
import robot_msgs.msg



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

    
def make_move_down_goal():
    goal = make_cartesian_trajectory_goal_world_frame(
        pos = [0.63, 0.0, 0.16],
        quat = [ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ],
        duration = 5.)
    goal.gains.append(make_cartesian_gains_msg(0., 20.))
    goal.force_guard.append(make_force_guard_msg(15.))
    return goal

def make_move_over_goal():
    goal = make_cartesian_trajectory_goal_world_frame(
        pos = [0.63, 0.05, 0.16],
        quat = [ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ],
        duration = 1.)
    goal.gains.append(make_cartesian_gains_msg(0., 20.))
    goal.force_guard.append(make_force_guard_msg(30.)) 
    return goal

def make_flip_over_goal():
    goal = make_cartesian_trajectory_goal_world_frame(
        pos = [0.63, 0.2, 0.3],
        quat = [ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ],
        duration = 0.5)
    goal.gains.append(make_cartesian_gains_msg(0., 20.))
    goal.force_guard.append(make_force_guard_msg(30.))    
    return goal

if __name__ == "__main__":
    rospy.init_node('sandboxx')
    
    robotSubscriber = JointStateSubscriber("/joint_states")
    rospy.sleep(1.0)
    
    print("Moving to start position")

    above_table_pre_grasp = [0.04486168762069299, 0.3256606458812486, -0.033502080520812445, -1.5769091802934694, 0.05899249087322813, 1.246379583616529, 0.38912999977004026]
    targetPosition = above_table_pre_grasp

    robotService = rosUtils.RobotService.makeKukaRobotService()
    success = robotService.moveToJointPosition(targetPosition, timeout=5)

    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    print("Got full state, starting control")

    # EE CONTROL VERSION
    client = actionlib.SimpleActionClient("plan_runner/CartesianTrajectory", robot_msgs.msg.CartesianTrajectoryAction)
    print "waiting for EE action server"
    client.wait_for_server()
    print "connected to EE action server"

    for goal in [make_move_down_goal(),
                 make_move_over_goal(),
                 make_flip_over_goal()]:
        print "sending goal"
        client.send_goal(goal)
        rospy.loginfo("waiting for CartesianTrajectory action result")
        client.wait_for_result()
        result = client.get_result()
