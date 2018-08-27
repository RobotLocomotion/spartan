import argparse
import time

import numpy as np

import rospy
import actionlib
import robot_msgs.msg
import robot_msgs.srv
import trajectory_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import std_srvs.srv

import robot_control.control_utils as control_utils
import spartan.utils.ros_utils as ros_utils

def test_joint_trajectory_action():
    client = actionlib.SimpleActionClient("plan_runner/JointTrajectory", robot_msgs.msg.JointTrajectoryAction)

    print "waiting for server"
    client.wait_for_server()
    print "connected to server"

    joint_position = [0.5]*7
    goal = robot_msgs.msg.JointTrajectoryGoal()
    goal.trajectory = make_joint_trajectory_msg(joint_position)

    print "sending goal"
    client.send_goal(goal)

    rospy.loginfo("waiting for JointTrajectory action result")
    client.wait_for_result()
    result = client.get_result()
    print "result:", result

# moves the "penetrate_baymax" position
def test_joint_trajectory_action_with_force_guard():
    
    # penetrate baymax position
    joint_position = [0.004693801442017107, 0.9628760254043379, -0.05429710750841024, -1.4038225779201545, 0.10079011175127535, 0.7508584287857891, 0.29501057123920577]

    # above baymax position


    goal = robot_msgs.msg.JointTrajectoryGoal()
    goal.trajectory = make_joint_trajectory_msg(joint_position)
    goal.force_guard.append(make_force_guard_msg())


    client = actionlib.SimpleActionClient("plan_runner/JointTrajectory", robot_msgs.msg.JointTrajectoryAction)

    print "waiting for server"
    client.wait_for_server()
    print "connected to server" 

    print "sending goal"
    client.send_goal(goal)   

    rospy.loginfo("waiting for JointTrajectory action result")
    client.wait_for_result()
    result = client.get_result()
    print "result:", result
    

def make_joint_trajectory_msg(joint_position):
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = control_utils.getIiwaJointNames()
    num_joints = 7

    traj_start = trajectory_msgs.msg.JointTrajectoryPoint()
    traj_start.positions = [0] * 7
    traj_start.time_from_start = rospy.Duration(0.0)


    traj_end = trajectory_msgs.msg.JointTrajectoryPoint()
    traj_end.positions = joint_position
    traj_end.time_from_start = rospy.Duration(3.0)

    traj.points.append(traj_start)
    traj.points.append(traj_end)

    return traj

def test_cartesian_trajectory_action(move_type="gripper_frame"):
    client = actionlib.SimpleActionClient("plan_runner/CartesianTrajectory", robot_msgs.msg.CartesianTrajectoryAction)

    print "waiting for server"
    client.wait_for_server()
    print "connected to server"

    if move_type == "gripper_frame":
        goal = make_cartesian_trajectory_goal_gripper_frame()
    if move_type == "world_frame":
        goal = make_cartesian_trajectory_goal_world_frame()

    goal.gains.append(make_cartesian_gains_msg())
    goal.force_guard.append(make_force_guard_msg())
    

    print "sending goal"
    client.send_goal(goal)

    rospy.loginfo("waiting for CartesianTrajectory action result")
    client.wait_for_result()
    result = client.get_result()
    print "result:", result


def make_cartesian_trajectory_goal_gripper_frame():
    goal = robot_msgs.msg.CartesianTrajectoryGoal()
    traj = goal.trajectory

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
    xyz_knot.point.x = 0.1
    xyz_knot.point.y = 0.0
    xyz_knot.point.z = 0.0

    traj.xyz_points.append(xyz_knot)

    traj.ee_frame_id = ee_frame_id

    traj.time_from_start.append(rospy.Duration(0.0))
    traj.time_from_start.append(rospy.Duration(4.0))


    return goal

def make_cartesian_trajectory_goal_world_frame():

    # (array([0.588497  , 0.00716426, 0.5159925 ]), array([ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ]))

    pos = [0.588497  , 0.00716426, 0.5159925]
    quat = [ 0.70852019, -0.15500524,  0.67372875,  0.1416407 ]

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
    traj.time_from_start.append(rospy.Duration(2.0))

    quat_msg = geometry_msgs.msg.Quaternion()
    quat_msg.w = quat[0]
    quat_msg.x = quat[1]
    quat_msg.y = quat[2]
    quat_msg.z = quat[3]

    traj.quaternions.append(quat_msg)

    return goal

def make_cartesian_gains_msg():
    msg = robot_msgs.msg.CartesianGain()

    kp_rot = 5
    msg.rotation.x = kp_rot
    msg.rotation.y = kp_rot
    msg.rotation.z = kp_rot

    kp_trans = 10
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

def test_joint_space_streaming():
    rospy.wait_for_service("plan_runner/init_joint_space_streaming")
    sp = rospy.ServiceProxy('plan_runner/init_joint_space_streaming',
        robot_msgs.srv.StartStreamingPlan)
    init = robot_msgs.srv.StartJointSpaceStreamingPlanRequest()
    init.force_guard.append(make_force_guard_msg())
    print sp(init)
    pub = rospy.Publisher('plan_runner/joint_space_streaming_setpoint',
        sensor_msgs.msg.JointState, queue_size=1)
    robotSubscriber = ros_utils.JointStateSubscriber("/joint_states")
    print("Waiting for full kuka state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    print("got full state")

    start_time = time.time()
    while (time.time() - start_time < 1.):
        current_joint_positions = robotSubscriber.get_position_vector_from_joint_names(control_utils.getIiwaJointNames())
    
        new_msg = sensor_msgs.msg.JointState()
        new_msg.name = control_utils.getIiwaJointNames()
        new_msg.position = current_joint_positions
        new_msg.velocity = np.zeros(7)
        new_msg.effort = np.zeros(7)
        new_msg.position[0] += 0.01
        pub.publish(new_msg)

    rospy.wait_for_service("plan_runner/stop_plan")
    sp = rospy.ServiceProxy('plan_runner/stop_plan',
        std_srvs.srv.Trigger)
    init = std_srvs.srv.TriggerRequest()
    print sp(init)

def test_task_space_streaming():
    rospy.wait_for_service("plan_runner/init_task_space_streaming")
    sp = rospy.ServiceProxy('plan_runner/init_task_space_streaming',
        robot_msgs.srv.StartStreamingPlan)
    init = robot_msgs.srv.StartStreamingPlanRequest()
    init.force_guard.append(make_force_guard_msg())
    print sp(init)
    pub = rospy.Publisher('plan_runner/task_space_streaming_setpoint',
        robot_msgs.msg.CartesianGoalPoint, queue_size=1)
    robotSubscriber = ros_utils.JointStateSubscriber("/joint_states")
    print("Waiting for full kuka state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    print("got full state")

    start_time = time.time()
    new_msg = robot_msgs.msg.CartesianGoalPoint()
    new_msg.xyz_point.header.frame_id = "iiwa_link_ee"
    new_msg.xyz_point.point.x = 0.0
    new_msg.xyz_point.point.y = 0.01
    new_msg.xyz_point.point.z = 0.0
    new_msg.xyz_d_point.x = 0.
    new_msg.xyz_d_point.y = 0.
    new_msg.xyz_d_point.z = 0.0
    new_msg.quaternion.w = 1.
    new_msg.quaternion.x = 0.
    new_msg.quaternion.y = 0.
    new_msg.quaternion.z = 0.
    new_msg.gain = make_cartesian_gains_msg()
    new_msg.ee_frame_id = "iiwa_link_ee"
    while (time.time() - start_time < 1.0):
        pub.publish(new_msg)

    rospy.wait_for_service("plan_runner/stop_plan")
    sp = rospy.ServiceProxy('plan_runner/stop_plan',
        std_srvs.srv.Trigger)
    init = std_srvs.srv.TriggerRequest()
    print sp(init)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--movement", type=str,
        help="(optional) type of movement, can be gripper_frame or world_frame", default="gripper_frame")
    rospy.init_node("test_plan_runner")
    args = parser.parse_args()
    test_joint_trajectory_action()
    # test_cartesian_trajectory_action(move_type=args.movement)
    # test_joint_trajectory_action_with_force_guard()
    #test_joint_space_streaming()
    test_task_space_streaming()