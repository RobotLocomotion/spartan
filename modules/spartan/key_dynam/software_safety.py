import numpy as np


class SoftwareSafety(object):

    def __init__(self):
        self._vel_limit = 0.3
        pass

    def check_message(self, msg):
        x = msg.setpoint_linear_velocity.x
        y = msg.setpoint_linear_velocity.y
        v = np.array([x,y])

        norm = np.linalg.norm(v)
        if norm < self._vel_limit:
            return True
        else:
            return False
