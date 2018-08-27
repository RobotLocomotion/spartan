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



def make_cartesian_trajectory_goal_world_frame():

    # (array([0.588497  , 0.00716426, 0.5159925 ]), array([ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ]))

    # Barely touching tabletop
    pos = [0.55, 0.0, 0.13]
    quat = [ 1.0, 0.0,  0.0,  0.0 ]

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
    traj.time_from_start.append(rospy.Duration(5.0))

    quat_msg = geometry_msgs.msg.Quaternion()
    quat_msg.w = quat[0]
    quat_msg.x = quat[1]
    quat_msg.y = quat[2]
    quat_msg.z = quat[3]

    traj.quaternions.append(quat_msg)

    return goal

def make_cartesian_trajectory_goal_gripper_frame():
    # (array([0.588497  , 0.00716426, 0.5159925 ]), array([ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ]))

    # Barely touching tabletop
    pos = [0.0, 0.0, 0.0]
    quat = [ 1.0, 0.0,  0.0,  0.0 ]

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
    traj.time_from_start.append(rospy.Duration(0.25))

    quat_msg = geometry_msgs.msg.Quaternion()
    quat_msg.w = quat[0]
    quat_msg.x = quat[1]
    quat_msg.y = quat[2]
    quat_msg.z = quat[3]

    traj.quaternions.append(quat_msg)

    return goal

def make_cartesian_gains_msg():
    msg = robot_msgs.msg.CartesianGain()

    kp_rot = 0
    msg.rotation.x = kp_rot
    msg.rotation.x = kp_rot
    msg.rotation.x = kp_rot

    kp_trans = 0.1
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

def make_joint_trajectory(q, v):
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = control_utils.getIiwaJointNames()
    num_joints = 7

    traj_start = trajectory_msgs.msg.JointTrajectoryPoint()
    traj_start.positions = q
    traj_start.velocities = v
    traj_start.time_from_start = rospy.Duration(0.0)


    traj_end = trajectory_msgs.msg.JointTrajectoryPoint()
    traj_end.positions = q+v
    traj_end.velocities = v
    traj_end.time_from_start = rospy.Duration(0.2)

    traj.points.append(traj_start)
    traj.points.append(traj_end)

    return traj

    

if __name__ == "__main__":
    rospy.init_node('sandboxx')
    
    robotSubscriber = JointStateSubscriber("/joint_states")
    print("Waiting for full kuka state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    print("got full state")

    print("Moving to start position")
    above_table_pre_grasp = [0.04486168762069299, 0.3256606458812486, -0.033502080520812445, -1.5769091802934694, 0.05899249087322813, 1.246379583616529, 0.38912999977004026]
    targetPosition = above_table_pre_grasp

    robotService = rosUtils.RobotService.makeKukaRobotService()
    success = robotService.moveToJointPosition(targetPosition, timeout=5)

    print("Starting control")

    # EE CONTROL VERSION
    '''
    client = actionlib.SimpleActionClient("plan_runner/CartesianTrajectory", robot_msgs.msg.CartesianTrajectoryAction)
    print "waiting for server"
    client.wait_for_server()
    print "connected to server"

    while(1):
        current_joint_positions = robotSubscriber.get_position_vector_from_joint_names(control_utils.getIiwaJointNames())
        print "Joint positions: ", current_joint_positions

        goal = make_cartesian_trajectory_goal_gripper_frame()
        goal.gains.append(make_cartesian_gains_msg())
        goal.force_guard.append(make_force_guard_msg())

        print "sending goal"
        client.send_goal(goal)

        rospy.loginfo("waiting for CartesianTrajectory action result")
        client.wait_for_result()
        result = client.get_result()
    '''

    # JOINT TRAJECTORY VERSION

    rospy.wait_for_service("plan_runner/init_joint_space_streaming")
    sp = rospy.ServiceProxy('plan_runner/init_joint_space_streaming',
        robot_msgs.srv.StartJointSpaceStreamingPlan)
    init = robot_msgs.srv.StartJointSpaceStreamingPlanRequest()
    #init.force_guard.append(make_force_guard_msg())
    sp(init)

    pub = rospy.Publisher('plan_runner/joint_space_streaming_setpoint',
        sensor_msgs.msg.JointState, queue_size=1)
    
    smoothed_position_est = None
    try:
        while(1):
            current_joint_positions = robotSubscriber.get_position_vector_from_joint_names(control_utils.getIiwaJointNames())
            current_joint_velocities = robotSubscriber.get_velocity_vector_from_joint_names(control_utils.getIiwaJointNames())
            current_joint_efforts = robotSubscriber.get_effort_vector_from_joint_names(control_utils.getIiwaJointNames())
            if smoothed_position_est is None:
                smoothed_position_est = current_joint_positions
            else:
                smoothed_position_est = smoothed_position_est*0.5 + current_joint_positions*0.5

            print smoothed_position_est, current_joint_velocities, current_joint_efforts
            new_msg = sensor_msgs.msg.JointState()
            new_msg.name = control_utils.getIiwaJointNames()
            new_msg.position = smoothed_position_est
            new_msg.velocity = current_joint_velocities
            new_msg.effort = current_joint_efforts*0.

            pub.publish(new_msg)
            rospy.sleep(0.005)

    except Exception as e:
        print("Error: ", e)
        rospy.wait_for_service("plan_runner/stop_plan")
        print("Trying to stop current plan")
        sp = rospy.ServiceProxy('plan_runner/stop_plan',
            std_srvs.srv.Trigger)
        init = std_srvs.srv.TriggerRequest()
        print sp(init)