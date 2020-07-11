import numpy as np

# ros
import tf2_ros
import rospy

from spartan.utils import ros_utils

class SoftwareSafety(object):

    def __init__(self,
                 tf_buffer=None,
                 world_frame="base",
                 end_effector_frame="iiwa_link_ee",
                 pos_min=None,
                 pos_max=None,
                 ):
        self._vel_limit = 0.4

        if tf_buffer is None:
            tf_buffer = tf2_ros.Buffer()

        self._tf_buffer = tf_buffer
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.world_frame = world_frame
        self.end_effector_frame = end_effector_frame

        if pos_min is None:
            # pos_min = np.array([0.47, -0.3631, 0])
            pos_min = np.array([0.4, -0.3631, 0]) # needed for curved traj

        self._pos_min = pos_min

        if pos_max is None:
            pos_max = np.array([0.8, 0.36, 1.0])

        self._pos_max = pos_max


    def check_ee_pos(self):
        pos, quat = ros_utils.poseFromROSTransformMsg(
            self._tf_buffer.lookup_transform(self.world_frame, self.end_effector_frame, rospy.Time()).transform)

        pos = np.array(pos)
        valid = (self._pos_min < pos).all() and (pos < self._pos_max).all()

        return valid


    def check_message(self, msg):

        ee_pos_ok = self.check_ee_pos()
        if not ee_pos_ok:
            return False

        x = msg.setpoint_linear_velocity.x
        y = msg.setpoint_linear_velocity.y
        v = np.array([x,y])

        norm = np.linalg.norm(v)
        if norm < self._vel_limit:
            return True
        else:
            return False
