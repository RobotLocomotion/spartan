from director import consoleapp
from director import lcmUtils
import drake as lcmdrake
import numpy as np


statusMsg1 = None
statusMsg2 = None
msg = lcmdrake.lcmt_iiwa_status()


def publishStatus():

    if None in (statusMsg1, statusMsg2):
        return

    for field in ['joint_position_measured',
                   'joint_position_commanded',
                   'joint_position_ipo',
                   'joint_torque_measured',
                   'joint_torque_commanded',
                   'joint_torque_external']:
        setattr(msg, field, list(getattr(statusMsg1, field)) + list(getattr(statusMsg2, field)))

    msg.utime = max(statusMsg1.utime, statusMsg2.utime)
    msg.num_joints = len(msg.joint_position_measured)
    lcmUtils.publish('IIWA_STATUS', msg)


def onIiwaStatus1(msg):
    global statusMsg1
    statusMsg1 = msg
    publishStatus()

def onIiwaStatus2(msg):
    global statusMsg2
    statusMsg2 = msg
    publishStatus()


subscriber = lcmUtils.addSubscriber('IIWA_STATUS_1', lcmdrake.lcmt_iiwa_status, onIiwaStatus1)
subscriber = lcmUtils.addSubscriber('IIWA_STATUS_2', lcmdrake.lcmt_iiwa_status, onIiwaStatus2)

consoleapp.ConsoleApp.start()
