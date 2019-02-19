# system
import numpy as np

# ROS
import rospy
import geometry_msgs.msg


# spartan ROS
import robot_msgs.msg


"""
A set of utilities related to generating plans/controllers for the robot
"""

def make_cartesian_gains_msg(kp_rot=5, kp_trans=10):
    """
    Makes cartesian gains message with specified gains
    """

    msg = robot_msgs.msg.CartesianGain()

    msg.rotation.x = kp_rot
    msg.rotation.y = kp_rot
    msg.rotation.z = kp_rot

    msg.translation.x = kp_trans
    msg.translation.y = kp_trans
    msg.translation.z = kp_trans

    return msg

def make_force_guard_msg(force_vector=None, body_frame="iiwa_link_ee", expressed_in_frame="iiwa_link_ee"):
    """
    Convenience method for making a force guard message.
    :param force_vector: The force vector (in the frame iiwa_link_ee),
     any force with a magnitude larger than this will trigger a stop
    :param body_frame: body frame at which this force is applied
    :param expressed_in_frame: frame in which this force is expressed (usually body or world)
    :return:
    """

    msg = robot_msgs.msg.ForceGuard()
    external_force = robot_msgs.msg.ExternalForceGuard()

    if force_vector is None:
        20 * np.array([-1, 0, 0])


    external_force.force.header.frame_id = expressed_in_frame
    external_force.body_frame = body_frame
    external_force.force.vector.x = force_vector[0]
    external_force.force.vector.y = force_vector[1]
    external_force.force.vector.z = force_vector[2]

    msg.external_force_guards.append(external_force)

    return msg


def make_cartesian_trajectory_goal(xyz_goal, ee_frame_id, expressed_in_frame, speed=0.01, duration=None):
    """
    Keeps the orientation constant. Moves to xyz_goal expressed in expressed_in_frame.

    expressed_in_frame should be a ros TF frame name

    :param xyz_goal: xyz goal in expressed_in_frame
    :type: np array of size [3,]
    :param: ee_frame_id: The frame that will move to the goal
    :param speed: Speed of specified link, in meters/second
    :return: robot_msgs.msg.CartesianTrajectoryGoal
    """
    goal = robot_msgs.msg.CartesianTrajectoryGoal()
    traj = goal.trajectory



    # the first knot point always gets replaced by the current
    xyz_knot = geometry_msgs.msg.PointStamped()
    xyz_knot.header.frame_id = expressed_in_frame
    xyz_knot.point.x = 0
    xyz_knot.point.y = 0
    xyz_knot.point.z = 0
    traj.xyz_points.append(xyz_knot)

    xyz_knot = geometry_msgs.msg.PointStamped()
    xyz_knot.header.frame_id = expressed_in_frame
    xyz_knot.point.x = xyz_goal[0]
    xyz_knot.point.y = xyz_goal[1]
    xyz_knot.point.z = xyz_goal[2]

    traj.xyz_points.append(xyz_knot)

    traj.ee_frame_id = ee_frame_id


    if duration is None:
        duration = max(np.linalg.norm(xyz_goal) / (1.0 * speed), 0.1)

    traj.time_from_start.append(rospy.Duration(0.0))
    traj.time_from_start.append(rospy.Duration(duration))

    return goal