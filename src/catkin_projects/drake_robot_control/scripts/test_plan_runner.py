import argparse


import rospy
import actionlib
import robot_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg

import robot_control.control_utils as control_utils


def test_joint_trajectory_action():
    client = actionlib.SimpleActionClient("plan_runner/JointTrajectory", robot_msgs.msg.JointTrajectoryAction)

    print "waiting for server"
    client.wait_for_server()
    print "connected to server"

    goal = robot_msgs.msg.JointTrajectoryGoal()
    goal.trajectory = make_joint_trajectory_msg()

    print "sending goal"
    client.send_goal(goal)

    rospy.loginfo("waiting for JointTrajectory action result")
    client.wait_for_result()
    result = client.get_result()
    print "result:", result
    

def make_joint_trajectory_msg():
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = control_utils.getIiwaJointNames()
    num_joints = 7

    traj_start = trajectory_msgs.msg.JointTrajectoryPoint()
    traj_start.positions = [0] * 7
    traj_start.time_from_start = rospy.Duration(0.0)


    traj_end = trajectory_msgs.msg.JointTrajectoryPoint()
    traj_end.positions = [0.5] * 7
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
    xyz_knot.point.x = 0.2
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
    msg.rotation.x = kp_rot
    msg.rotation.x = kp_rot

    kp_trans = 10
    msg.translation.x = kp_trans
    msg.translation.y = kp_trans
    msg.translation.z = kp_trans

    return msg



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--movement", type=str,
        help="(optional) type of movement, can be gripper_frame or world_frame", default="gripper_frame")
    rospy.init_node("test_plan_runner")
    args = parser.parse_args()
    # test_joint_trajectory_action()
    test_cartesian_trajectory_action(move_type=args.movement)