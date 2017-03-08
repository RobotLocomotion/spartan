from director import lcmUtils
import bot_core as lcmbotcore
import json


baseJoints = ['base_x', 'base_y', 'base_z', 'base_roll', 'base_pitch', 'base_yaw']

armJoints = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'upper_arm_roll_joint',
    'elbow_flex_joint',
    'forearm_roll_joint',
    'wrist_flex_joint',
    'wrist_roll_joint']

headJoints = ['head_pan_joint', 'head_tilt_joint']

gripperJoints = ['gripper_l_finger_joint',  'gripper_r_finger_joint']
#gripperJoints = ['gripper_l_parallel_root_joint', 'gripper_r_parallel_root_joint']

torsoJoints = ['torso_lift_joint']


def getSidePrefix(side):
    return {'left':'l_', 'right':'r_'}[side.lower()]


def getJointController():
    return robotSystem.robotStateJointController


def getJointIndices(names):
    jointController = getJointController()
    for name in names:
        if name not in jointController.jointNames:
            print 'unknown joint:', name
    return [jointController.jointNames.index(name) for name in names]


def setJointPositions(names, positions):
    getJointController().q[getJointIndices(names)] = positions


def setBaseJoints(joints):
    setJointPositions(['base_x', 'base_y', 'base_yaw'], joints)


def setHeadJoints(joints):
    setJointPositions(headJoints, joints)


def setTorsoJoints(joints):
    setJointPositions(torsoJoints, joints)


def setArmJoints(side, joints):
    prefix = getSidePrefix(side)
    names = [prefix+name for name in armJoints]
    print names
    setJointPositions(names, joints)


def setGripperJoints(side, joints):
    prefix = getSidePrefix(side)
    names = [prefix+name for name in gripperJoints]
    setJointPositions(names, [joints + joints])


def setRobotConf(conf):

    jointController = getJointController()
    jointController.q = np.zeros(len(jointController.jointNames))
    jointController.push()


    setArmJoints('left', conf['pr2LeftArm'])
    setArmJoints('right', conf['pr2RightArm'])
    setHeadJoints(conf['pr2Head'])
    setTorsoJoints(conf['pr2Torso'])
    setBaseJoints(conf['pr2Base'])
    #setGripperJoints('left', conf['pr2LeftGripper'])
    #setGripperJoints('right', conf['pr2RightGripper'])

    jointController.push()


def onPR2JointConfMessage(msg):
    conf = json.loads(msg.command_data)
    setRobotConf(conf)


def initLcmSubscriber():
    lcmUtils.addSubscriber('BHPN_PR2_JOINTCONF', lcmbotcore.viewer_command_t, onPR2JointConfMessage)


def test():

    import os
    import pydrake
    from director import roboturdf

    filename = os.path.join(pydrake.getDrakePath(), 'examples/PR2/pr2.urdf')
    assert os.path.isfile(filename)

    robotModel, jointController = roboturdf.loadRobotModel(urdfFile=filename, view=view, useConfigFile=False)


    conf = {
        'pr2LeftGripper': [0.07],
        'pr2RightArm': [-0.91698, 0.042600, -1.5, -2.01531, -1.57888, -1.65300, -2.04511],
        'pr2Base': [0.0, 0.0, 0.0],
        'pr2Torso': [0.3],
        'pr2RightGripper': [0.07],
        'pr2Head': [0.0, 0.0],
        'pr2LeftArm': [2.1, 1.29, 0.0, -0.15, 0.0, -0.1, 0.0]
        }

    setRobotConf(conf)


if __name__ == '__main__':

    initLcmSubscriber()
    #test()
