from director import lcmUtils
import bot_core as lcmbotcore
import json
import numpy as np


class BHPNTranslator(object):
    '''
    This class translates a JointConf from BHPN into a robot state for the
    PR2 model loaded with drake.
    '''

    armJoints = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'upper_arm_roll_joint',
        'elbow_flex_joint',
        'forearm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint']
    baseJoints = ['base_x', 'base_y', 'base_yaw']
    headJoints = ['head_pan_joint', 'head_tilt_joint']
    gripperJoints = ['gripper_l_finger_joint',  'gripper_r_finger_joint']
    torsoJoints = ['torso_lift_joint']

    def __init__(self, jointController):
        self.jointController = jointController
        self.initLcmSubscriber()

    def getSidePrefix(self, side):
        return {'left':'l_', 'right':'r_'}[side.lower()]

    def getJointController(self):
        return self.jointController

    def getJointIndices(self, names):
        jointController = self.getJointController()
        for name in names:
            if name not in jointController.jointNames:
                print 'unknown joint:', name
        return [jointController.jointNames.index(name) for name in names]

    def setJointPositions(self, names, positions):
        self.getJointController().q[self.getJointIndices(names)] = positions

    def setBaseJoints(self, joints):
        self.setJointPositions(self.baseJoints, joints)

    def setHeadJoints(self, joints):
        self.setJointPositions(self.headJoints, joints)

    def setTorsoJoints(self, joints):
        self.setJointPositions(self.torsoJoints, joints)

    def setArmJoints(self, side, joints):
        prefix = self.getSidePrefix(side)
        names = [prefix+name for name in self.armJoints]
        self.setJointPositions(names, joints)

    def setGripperJoints(self, side, joints):
        prefix = self.getSidePrefix(side)
        names = [prefix+name for name in self.gripperJoints]
        self.setJointPositions(names, [joints + joints])

    def setRobotConf(self, conf):

        jointController = self.getJointController()
        jointController.q = np.zeros(len(jointController.jointNames))
        jointController.push()

        self.setArmJoints('left', conf['pr2LeftArm'])
        self.setArmJoints('right', conf['pr2RightArm'])
        self.setHeadJoints(conf['pr2Head'])
        self.setTorsoJoints(conf['pr2Torso'])
        self.setBaseJoints(conf['pr2Base'])

        # not sure these are mapped correctly, so not setting them for now
        #setGripperJoints('left', conf['pr2LeftGripper'])
        #setGripperJoints('right', conf['pr2RightGripper'])

        jointController.push()

    def onJointConfMessage(self, msg):
        conf = json.loads(msg.command_data)
        self.setRobotConf(conf)

    def initLcmSubscriber(self, channel='BHPN_PR2_JOINTCONF'):
        self.sub = lcmUtils.addSubscriber(channel, lcmbotcore.viewer_command_t, self.onJointConfMessage)


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

    t = BHPNTranslator(jointController)
    t.setRobotConf(conf)


if __name__ == '__main__':
    test()
