import unittest
import subprocess
import psutil
import sys
import os
import numpy as np
import time
import socket
# Using pygame for gamepad interface
import pygame

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

def do_main():
    rospy.init_node('gamepad_teleop', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    robotSubscriber = JointStateSubscriber("/joint_states")
    
    robotSubscriber = ros_utils.JointStateSubscriber("/joint_states")
    rospy.loginfo("Waiting for full kuka state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    rospy.loginfo("got full state")

    rospy.loginfo("Grabbing controller...")
    pygame.init()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    rospy.loginfo("Found %d controllers" % len(joysticks))
    if len(joysticks) == 0:
        rospy.logerr("Didn't find a controller :(.")
        sys.exit(-1)
    joystick = joysticks[0]
    joystick.init()
    rospy.loginfo("Using joystick %s" % joystick.get_name())

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

    tf_ee = None

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
        last_update_time = time.time()
        while not rospy.is_shutdown():
            #for axis_i in range(joystick.get_numaxes()):
            #    rospy.loginfo("Axis %d: %f" % (axis_i, joystick.get_axis(axis_i)))
            #for button_i in range(joystick.get_numbuttons()):
            #    rospy.loginfo("Button %d: %f" % (button_i, joystick.get_button(button_i)))
            #time.sleep(0.5)

            # Gamepad: Logitech 310
            # Left stick: Axis 7 +1 is down
            #             Axis 6 +1 is right
            # Right stick: Axis 3 +1 is right
            #              Axis 4 +1 is down 
            # Left bumper: Button 4
            # Right bumper: Button 5
            gripper_dt = time.time() - last_gripper_update_time
            dt = time.time() - last_update_time
            pygame.event.pump()
            if gripper_dt > 0.2:
                last_gripper_update_time = time.time()
                gripper_goal_pos += (joystick.get_button(5) - joystick.get_button(4))*dt*0.05
                gripper_goal_pos = max(min(gripper_goal_pos, 0.1), 0.0)
                handDriver.sendGripperCommand(gripper_goal_pos, speed=0.1, timeout=0.01)
                print "Gripper goal pos: ", gripper_goal_pos
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

            if dt > 0.01:
                last_update_time = time.time()
                if tf_ee is None:
                    tf_ee = tf_matrix_from_pose(current_pose_ee)
                    

                tf_ee[0, 3] += -1.*joystick.get_axis(7)*dt*0.1
                tf_ee[1, 3] += -1.*joystick.get_axis(6)*dt*0.1
                tf_ee[2, 3] += -1.*joystick.get_axis(4)*dt*0.1

                target_trans_ee = tf_ee[:3, 3]
                target_quat_ee = transformations.quaternion_from_matrix(tf_ee)

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

    except Exception as e:
        print "Suffered exception ", e


if __name__ == "__main__":
    do_main()
