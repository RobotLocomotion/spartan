__author__ = 'manuelli'
import numpy as np
import collections
import yaml
import os

from collections import namedtuple

from director.timercallback import TimerCallback
from director import robotstate
import drake as lcmdrake
from director import lcmUtils
from director import utime as utimeUtil



# here q should be the director pose
def getForceDirectionInWorld(q, robotStateModel, linkName, forceLocation, forceDirection):
    forceDirection = forceDirection/np.linalg.norm(forceDirection)
    linkToWorld = robotStateModel.getLinkFrame(linkName)
    forceLocationInWorld = np.array(linkToWorld.TransformPoint(forceLocation))
    forceDirectionInWorld = np.array(linkToWorld.TransformDoubleVector(forceDirection))

    return forceDirectionInWorld, forceLocationInWorld


def removeElementsFromList(givenList, toRemove):
    for x in toRemove:
        if x in givenList:
            givenList.remove(x)


def getPerpendicularVector(vec):
    perpendicularVectorr = np.cross(np.array([1,0,0]), vec)
    eps = 1e-4
    if (np.linalg.norm(perpendicularVector) < eps):
        perpendicularVector = perpendicularVector = np.cross(np.array([0,0,1]), vec)

    perpendicularVector = perpendicularVector/np.linalg.norm(perpendicularVector)
    return perpendicularVector

# d should be a dict. d.keys() will be the fields of
def createNamedTupleFromDict(d, name='Default'):
    x = namedtuple(name, d.keys())

    for key, val in d.iteritems():
        setattr(x,key,val)

    return x

def loadConfig(config_filename="contact_particle_filter_config.yaml"):
    spartan_source_dir = os.getenv("SPARTAN_SOURCE_DIR")
    fullFileName = spartan_source_dir + '/src/ContactParticleFilter/config/' + config_filename

    stream = file(fullFileName)
    config = yaml.load(stream)
    return config


class DequePeak(collections.deque):

    def __init__(self):
        collections.deque.__init__(self)

    def peakRight(self):
        peakVal = None
        try:
            peakVal = self.pop()
            self.append(peakVal)
        except IndexError:
            peakVal = None

        return peakVal

    def peakLeft(self):
        peakVal = None
        try:
            peakVal = self.popleft()
            self.appendleft(peakVal)
        except IndexError:
            peakVal = None

        return peakVal


class EstRobotStatePublisher(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.timer = TimerCallback(targetFps=25)
        self.timer.callback = self.publishEstRobotState

    def publishEstRobotState(self):
        q = self.robotSystem.robotStateJointController.q
        stateMsg = robotstate.drakePoseToRobotState(q)
        stateMsg.utime = utimeUtil.getUtime()
        lcmUtils.publish("EST_ROBOT_STATE", stateMsg)

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()


# translates director pose to our pose
class RobotPoseTranslator(object):

    def __init__(self, directorDrakeModel, drakeModel):
        directorJointNames = directorDrakeModel.getJointNames()
        jointNames = drakeModel.getJointNames()

        self.numJoints = len(jointNames)
        self.jointIdxToDirectorJointIdx = [None]*self.numJoints

        for idx, name in enumerate(jointNames):
            if name in directorJointNames:
                self.jointIdxToDirectorJointIdx[idx] = directorJointNames.index(name)
            else:
                text = "joint (" + str(name) + ") exists in your model but not in " \
                                             "director model"
                raise ValueError(text)



    def translateDirectorPoseToRobotPose(self, q_director):
        q = np.zeros(self.numJoints)
        for idx in xrange(self.numJoints):
            q[idx] = q_director[self.jointIdxToDirectorJointIdx[idx]]

        return q

