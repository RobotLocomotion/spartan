__author__ = 'manuelli'

import os

from director import ioUtils
from director import timercallback
class CPFTester:

    def __init__(self, externalForce):
        self.externalForce = externalForce
        self.timer = timercallback.TimerCallback(targetFps=0.1, callback=self.timerCallback)
        self.callbackCounter = 0

    def loadContactPointDict(self, filename='singleContactForces.out'):
        fullFilename = os.getenv('DRC_BASE') + '/software/control/residual_detector/python/data/' + filename
        self.savedForcesDict = ioUtils.readDataFromFile(fullFilename)

    def loadContactForceDataFromDict(self, key):
        self.forceData = self.savedForcesDict[key]
        self.forceDataKeys = self.forceData.keys()
        self.numForces = len(self.forceData)

    def timerCallback(self):
        self.addForce(self.callbackCounter)

    def addForce(self, callbackCounter):
        print "adding force " + str(callbackCounter)
        d = self.forceData[self.forceDataKeys[callbackCounter]]
        self.externalForce.addForce(d['linkName'], forceDirection=d['forceDirection'], forceMagnitude=d['forceMagnitude'],
                                    forceLocation=d['forceLocation'])

        self.callbackCounter += 1
        if self.numForces == self.callbackCounter:
            print "done adding forces"
            self.timer.stop()