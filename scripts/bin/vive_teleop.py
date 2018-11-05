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


def ro(quat):
    return [quat[1], quat[2], quat[3], quat[0]]

def tf_matrix_from_pose(pose):
    trans, quat = pose
    mat = transformations.quaternion_matrix(quat)
    mat[:3, 3] = trans
    return mat

def get_relative_tf_between_poses(pose_1, pose_2):
    tf_1 = tf_matrix_from_pose(pose_1)
    tf_2 = tf_matrix_from_pose(pose_2)
    return np.linalg.inv(tf_1).dot(tf_2)

def build_rbt_from_ros_environment():
    robot = RigidBodyTree()
    package_map = PackageMap()
    package_map.PopulateFromEnvironment("ROS_PACKAGE_PATH")
    base_dir = getDrakePath()
    weld_frame = None
    floating_base_type = FloatingBaseType.kFixed
    robot = RigidBodyTree()
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        rospy.get_param("/robot_description"),
        package_map,
        base_dir,
        floating_base_type,
        weld_frame,
        robot)
    return robot

def poseFromTFMessage(msg, id):
    for transform in msg.transforms:
        if transform.child_frame_id == id:
            return ros_utils.poseFromROSTransformMsg(transform.transform)
    return False

def do_main():
    rospy.init_node('sandbox', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    robotSubscriber = JointStateSubscriber("/joint_states")
    
    robotSubscriber = ros_utils.JointStateSubscriber("/joint_states")
    print("Waiting for full kuka state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    print("got full state")

    rightControllerState = SimpleSubscriber("/vive_right", sensor_msgs.msg.Joy)
    rightControllerState.start()
    print("Waiting for vive...")
    rightControllerState.waitForNextMessage()
    print("Got vive controller state")

    #init constants 
    move_button_id = 2 # press trackpad
    callibrate_button_id = 4 # side buttons
    trigger_axis_id = 0

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
    print sp(init)
    print("Started task space streaming")


    pub = rospy.Publisher('plan_runner/task_space_streaming_setpoint',
        robot_msgs.msg.CartesianGoalPoint, queue_size=1)

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
    start_pose_controller = None
    start_tf_ee = None

    #safe_space_bounds = np.array([
    #    [-0.1525049945545604, -0.29228035558728516, 0.13544803250537002],
    #    [-0.4769166944956231, -0.35229338435348867, 0.19769445898112134],
    #    [-0.457830238829753, 0.20935562473070765, 0.21958282208421853],
    #    [-0.11156388045357436, 0.19961179929244402, 0.26618401754649285],
    #    [-0.10225744036375771, 0.22933143089985508, 0.48444059628986263],
    #    [-0.14861448807866284, -0.41030619355456643, 0.4648083304072826],
    #    [-0.5242759491071456, -0.4147275210829423, 0.4948555294112139],
    #    [-0.4847194053597296, 0.27176780719677074, 0.45391525596908033],
    #    [-0.17358753502356636, 0.18660040610810102, 0.15775990744092278],
    #    [-0.45109038331994794, 0.20434001341514574, 0.09804323148032473],
    #    [-0.4716416007082929, -0.3620164988593509, 0.12965905105466402],
    #    [-0.16130783846258154, -0.3208282816424661, 0.109649432666865]])
    # Reasonable inner bounding box
    safe_space_lower = np.array([-0.45, -0.35, 0.125])
    safe_space_upper = np.array([-0.15, 0.2, 0.45])
    safe_space_violation = False
    last_safe_space_complaint = time.time() - 1000.

    def cleanup():
        rospy.wait_for_service("plan_runner/stop_plan")
        sp = rospy.ServiceProxy('plan_runner/stop_plan',
            std_srvs.srv.Trigger)
        init = std_srvs.srv.TriggerRequest()
        print sp(init)
        print("Done cleaning up and stopping streaming plan")

    frame_name = "iiwa_link_ee"
    # origin_tf, in the above EE frame
    origin_tf = transformations.euler_matrix(0.0, 0., 0.)
    origin_tf[0:3, 3] = np.array([0.15, 0.0, 0.0])
    origin_tf_inv = np.linalg.inv(origin_tf)

    rospy.on_shutdown(cleanup)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)
    try:
        last_gripper_update_time = time.time()
        last_move_pressed = False
        while not rospy.is_shutdown():
            # get controller data
            latest_state_msg = rightControllerState.last_message

            # Gripper 
            dt = time.time() - last_gripper_update_time
            if dt > 0.2:
                last_gripper_update_time = time.time()
                gripper_goal_pos = 0.1 * (1-latest_state_msg.axes[trigger_axis_id])
                handDriver.sendGripperCommand(gripper_goal_pos, speed=0.1)
                print "Gripper goal pos: ", gripper_goal_pos
                br.sendTransform(origin_tf[0:3, 3],
                                 ro(transformations.quaternion_from_matrix(origin_tf)),
                                 rospy.Time.now(),
                                 "origin_tf",
                                 frame_name)

            # get current tf from ros world to ee
            try:
                pose_current_ros_ee = ros_utils.poseFromROSTransformMsg(
                    tfBuffer.lookup_transform("base", frame_name, rospy.Time()).transform)
                tf_current_ros_ee = tf_matrix_from_pose(pose_current_ros_ee)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Troubling looking up tf...")
                rate.sleep()
                continue

            # get current tf from vr world to controller
            try:
                pose_current_vr_controller = ros_utils.poseFromROSTransformMsg(
                    tfBuffer.lookup_transform("world", "right_controller", rospy.Time()).transform)
                tf_current_vr_controller = tf_matrix_from_pose(pose_current_vr_controller)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Troubling looking up tf...")
                rate.sleep()
                continue

            # get tf from ros world to vr world (should be constant while moving)
            try:
                pose_ros_vr = ros_utils.poseFromROSTransformMsg(
                    tfBuffer.lookup_transform("base", "world", rospy.Time()).transform)
                tf_ros_vr = tf_matrix_from_pose(pose_ros_vr)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Troubling looking up tf...")
                rate.sleep()
                continue

            # Check if move button pressed
            move_pressed = (latest_state_msg.buttons[move_button_id] == 1)
            
            if move_pressed:
                if last_move_pressed is False:
                    print("begin motion")
                    tf_start_ros_ee = tf_current_ros_ee
                    tf_start_vr_controller = tf_current_vr_controller

                # calculate starting tf from controller to ee 
                tf_start_controller_vr = np.linalg.inv(tf_start_vr_controller)
                tf_vr_ros = np.linalg.inv(tf_ros_vr)
                tf_start_controller_ee = tf_start_controller_vr.dot(tf_vr_ros).dot(tf_start_ros_ee)

                # we want tf from controller to ee to be constant for intuitive control
                # thus, use this to find tf from ros world to target ee
                tf_target_ros_ee = tf_ros_vr.dot(tf_current_vr_controller).dot(tf_start_controller_ee)

                # calculate positions of controllers in ros frame
                tf_start_ros_controller = tf_ros_vr.dot(tf_start_vr_controller)
                tf_current_ros_controller = tf_ros_vr.dot(tf_current_vr_controller)

                # subtract to get movement vector
                controller_trans = tf_current_ros_controller[:3, 3] - tf_start_ros_controller[:3, 3]

                # convert tf to trans and quat
                target_trans_ee = tf_start_ros_ee[:3, 3] + controller_trans # start position + movement vector
                target_quat_ee = transformations.quaternion_from_matrix(tf_target_ros_ee)
                target_quat_ee = np.array(target_quat_ee) / np.linalg.norm(target_quat_ee)

                # publish target pose ase cartesian goal point
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
