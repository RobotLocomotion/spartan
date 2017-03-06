__author__ = 'manuelli'



# standard imports
import numpy as np
import PythonQt
import os
import os.path



# fakes an enum class
class FloatingBaseType:

    def __init__(self, floatingJointTypeString):
        if floatingJointTypeString not in ["FIXED", "ROLLPITCHYAW", "QUATERNION"]:
            raise ValueError('FloatingJointType must be one of [FIXED, ROLLPITCHYAW, QUATERNION]')
        self.floatingJointTypeString = floatingJointTypeString

    # returns a string associated with this enum type
    def getTypeAsString(self):
        return self.floatingJointTypeString


# helper class that wraps ddDrakeModel
class PythonDrakeModel(object):

    def __init__(self, floatingJointTypeString, filename):
        self.loadRobotModelFromURDFFilename(floatingJointTypeString, filename)
        self.jointNameToIdxMap = self.getJointNameToIdxMap()
        self.jointIdxToNameMap = self.getJointIdxToNameMap()
        self.jointNames = self.model.getJointNames()

    # filename should be relative to drake source director
    def loadRobotModelFromURDFFilename(self, floatingBaseTypeString, filename):
        spartan_source_dir = os.getenv('SPARTAN_SOURCE_DIR')

        # print "spartan_source_dir = ", spartan_source_dir
        # print "filename = ", filename
        filename = spartan_source_dir + filename

        # checks that this is valid floating joint type
        floatingBaseType = FloatingBaseType(floatingBaseTypeString)

        self.model = PythonQt.dd.ddDrakeModel()
        if not self.model.loadFromFile(filename, floatingBaseType.getTypeAsString()):
            print "failed to load model"

        self.nv = self.model.numberOfJoints()
        self.numJoints = self.model.numberOfJoints()

    def getJointNameToIdxMap(self):

        jointNames = self.model.getJointNames()

        jointMap = dict()

        for idx, jointName in enumerate(jointNames):
            jointName = str(jointName)
            jointMap[jointName] = idx

        return jointMap

    def getJointIdxToNameMap(self):
        jointNames = self.model.getJointNames()

        jointIdxToNameMap = [None]*len(jointNames)

        for idx, jointName in enumerate(jointNames):
            jointIdxToNameMap[idx] = str(jointName)

        return jointIdxToNameMap

    def getJointNameFromIdx(self, jointIdx):
        """
        Returns joint name given a joint idx
        :param jointIdx:
        :return: jointName
        """

        return self.jointIdxToNameMap[jointIdx]

    def extractDataFromMessage(self, msgJointNames, msgData):

        msgJointMap = {}
        for msgName, msgData in zip(msgJointNames, msgData):
            msgJointMap[msgName] = msgData


        data = np.zeros(self.numJoints)
        for jointName, idx in self.jointNameToIdxMap.iteritems():
            data[idx] = msgJointMap[jointName]


        return data


    # make sure you call setJointPositions(q) on the ddDrakeModel BEFORE you
    # call this method
    def geometricJacobian(self, base_body_or_frame_ind, end_effector_body_or_frame_id,
                          expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot=False):

        linkJacobianVec = np.array(self.model.geometricJacobian(base_body_or_frame_ind, end_effector_body_or_frame_id,
                          expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot));

        linkJacobian = linkJacobianVec.reshape(6,self.nv)

        return linkJacobian

    def testGeometricJacobian(self):
        q = np.zeros(self.nv)
        self.model.doKinematics(q,0*q,False, False)
        return self.geometricJacobian(0,1,1,0,False)