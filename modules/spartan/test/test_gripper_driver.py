import argparse
import time

import numpy as np

import rospy
import actionlib
import wsg_50_common.msg

from spartan.utils.schunk_driver import SchunkDriver


def open_gripper():
    client = actionlib.SimpleActionClient("/wsg50_driver/wsg50/gripper_control", wsg_50_common.msg.CommandAction)

    print "waiting for server"
    client.wait_for_server()
    print "connected to server"

    goal = wsg_50_common.msg.CommandGoal()
    goal.command.command_id = wsg_50_common.msg.Command.MOVE
    goal.command.width = 0.1
    goal.command.speed = 0.1
    goal.command.force = 100
    goal.command.stop_on_block = False


    client.send_goal(goal)
    rospy.loginfo("waiting for Command action result")
    client.wait_for_result()
    result = client.get_result()
    print "result:", result

def close_gripper():
    rospy.loginfo("closing gripper")
    client = actionlib.SimpleActionClient("/wsg50_driver/wsg50/gripper_control", wsg_50_common.msg.CommandAction)

    print "waiting for server"
    client.wait_for_server()
    print "connected to server"

    goal = wsg_50_common.msg.CommandGoal()
    goal.command.command_id = wsg_50_common.msg.Command.MOVE
    goal.command.width = 0.0
    goal.command.speed = 0.1
    goal.command.force = 80
    goal.command.stop_on_block = False

    print goal


    client.send_goal(goal)
    rospy.loginfo("waiting for Command action result")
    client.wait_for_result()
    result = client.get_result()
    print "result:", result

def gripper_has_object():
    schunk_driver = SchunkDriver()
    gripper_has_object = schunk_driver.gripper_has_object()
    print "gripper_has_object:", gripper_has_object



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    rospy.init_node("test_gripper_driver")
    parser.add_argument("-o", "--open", action="store_true",
        help="specifies to open gripper", default=False)

    args = parser.parse_args()
    if args.open:
        open_gripper()
    else:
        close_gripper()

    gripper_has_object()