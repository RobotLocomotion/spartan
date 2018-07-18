import rospy
import actionlib
import robot_msgs.msg
import trajectory_msgs.msg

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

if __name__ == "__main__":
    rospy.init_node("test_plan_runner")
    test_joint_trajectory_action()