import rospy
import actionlib
import robot_msgs.msg

def test_joint_trajectory_action():
    client = actionlib.SimpleActionClient("plan_runner/JointTrajectory", robot_msgs.msg.JointTrajectoryAction)

    print "waiting for server"
    client.wait_for_server()
    print "connected to server"

    goal = robot_msgs.msg.JointTrajectoryGoal()

    print "sending goal"
    client.send_goal(goal)

    print "received goal"


if __name__ == "__main__":
    rospy.init_node("test_plan_runner")
    test_joint_trajectory_action()