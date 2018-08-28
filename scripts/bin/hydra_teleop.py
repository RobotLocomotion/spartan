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

if __name__ == "__main__":
    rospy.init_node('sandbox', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    robotSubscriber = JointStateSubscriber("/joint_states")
    
    robotSubscriber = ros_utils.JointStateSubscriber("/joint_states")
    print("Waiting for full kuka state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    print("got full state")

    hydraSubscriber = SimpleSubscriber("/hydra_calib", razer_hydra.msg.Hydra)
    hydraSubscriber.start()
    print("Waiting for hydra startup...")
    hydraSubscriber.waitForNextMessage()
    print("Got hydra.")

    handDriver = SchunkDriver()
    gripper_goal_pos = 0.1
    handDriver.reset_and_home()
    handDriver.reset_and_home()

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
    start_pose_wand = None
    start_tf_ee = None

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

    try:
        last_gripper_update_time = time.time()
        while not rospy.is_shutdown():
            latest_hydra_msg = hydraSubscriber.waitForNextMessage()
            dt = time.time() - last_gripper_update_time
            if dt > 2.0:
                last_time = time.time()
                gripper_goal_pos += latest_hydra_msg.paddles[0].joy[0]*dt*0.1
                gripper_goal_pos = max(min(gripper_goal_pos, 0.1), 0.0)
                handDriver.sendGripperCommand(gripper_goal_pos, speed=1.0)
                print gripper_goal_pos

            br.sendTransform(origin_tf[0:3, 3],
                             ro(transformations.quaternion_from_matrix(origin_tf)),
                             rospy.Time.now(),
                             "origin_tf",
                             frame_name)

            try:
                current_pose_ee = ros_utils.poseFromROSTransformMsg(
                    tfBuffer.lookup_transform("base", frame_name, rospy.Time()).transform)
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

                # These expect quat in x y z w, rather than our normal w x y z
                br.sendTransform(start_pose_wand[0], ro(start_pose_wand[1]),
                                 rospy.Time.now(),
                                 "start_pose_wand",
                                 "base")
                br.sendTransform(current_pose_wand[0], ro(current_pose_wand[1]),
                                 rospy.Time.now(),
                                 "current_pose_wand",
                                 "base")
                br.sendTransform(start_pose_ee[0], ro(start_pose_ee[1]),
                                 rospy.Time.now(),
                                 "start_pose_ee",
                                 "base")

                hydra_tf = get_relative_tf_between_poses(start_pose_wand, current_pose_wand)

                # Target TF for the EE will be its start TF plus this offset
                rot_slerp_amount = 1.0
                trans_slerp_amount = 0.5

                tf_in_ee_frame = origin_tf.dot(hydra_tf).dot(origin_tf_inv)

                br.sendTransform(tf_in_ee_frame[0:3, 3],
                                 ro(transformations.quaternion_from_matrix(tf_in_ee_frame)),
                                 rospy.Time.now(),
                                 "tf_in_ee_frame",
                                 "base")

                start_tf_ee_inv = np.linalg.inv(start_tf_ee)
                target_tf_ee = start_tf_ee.copy()
                target_tf_ee[0:3, 3] += tf_in_ee_frame[0:3, 3] # copy position change in world frame
                target_tf_ee[0:3, 0:3] = tf_in_ee_frame[0:3, 0:3].dot(target_tf_ee[0:3, 0:3])

                br.sendTransform(target_tf_ee[0:3, 3],
                                 ro(transformations.quaternion_from_matrix(target_tf_ee)),
                                 rospy.Time.now(),
                                 "target_tf_ee",
                                 "base")

                target_trans_ee = trans_slerp_amount*target_tf_ee[:3, 3] + (1. - trans_slerp_amount)*np.array(start_pose_ee[0])
                target_quat_ee = transformations.quaternion_slerp(
                    np.array(start_pose_ee[1]),
                    transformations.quaternion_from_matrix(target_tf_ee),
                    rot_slerp_amount)
                target_quat_ee = np.array(target_quat_ee) / np.linalg.norm(target_quat_ee)

                br.sendTransform(target_trans_ee,
                                 ro(target_quat_ee),
                                 rospy.Time.now(),
                                 "target_tf_ee_interp",
                                 "base")

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
                rospy.sleep(0.01)

    except Exception as e:
        print "Suffered exception ", e
