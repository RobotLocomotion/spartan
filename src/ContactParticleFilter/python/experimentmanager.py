import os
import time

from director import ioUtils

import spartan.utils.utils as spartanUtils
from spartan.utils.taskrunner import TaskRunner

class ExperimentManager(object):

    def __init__(self, robotStateModel, robotStateJointController, linkSelection, externalForce, estRobotStatePublisher, configFilename=""):
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.linkSelection = linkSelection
        self.externalForce = externalForce
        self.estRobotStatePublisher = estRobotStatePublisher
        self.spartanSourceDir = spartanUtils.getSpartanSourceDir()
        self.loadConfig()
        self.loadForcesFromFile()
        self.experimentData = dict()
        self.taskRunner = TaskRunner()

    def loadConfig(self):
        filename =  os.path.join(self.spartanSourceDir,"src","ContactParticleFilter", "config", "experiments", "cpf_experiment_config.yaml")
        self.config = spartanUtils.getDictFromYamlFilename(filename)

    def loadForcesFromFile(self, filename=None):
        if filename is None:
            filename = "force_locations.out"

        fullFilename = os.path.join(self.spartanSourceDir,"src","ContactParticleFilter", "config", "experiments", filename)

        spartan_source_dir = spartanUtils.getSpartanSourceDir()
        # fullFilename = spartan_source_dir + self.options['data']['initialParticleLocations']
        self.savedForceLocations = ioUtils.readDataFromFile(fullFilename)

    def addExternalForce(self, forceName='iiwa_link_7_1'):
        d = self.savedForceLocations[forceName]
        linkName = d['linkName']
        forceLocation = d['forceLocation']
        forceDirection = d['forceDirection']
        # forceMagnitude = d['forceMagnitude']
        forceMagnitude = self.config['force_magnitude']
        self.externalForce.addForce(linkName,
                                    forceDirection=forceDirection,
                                    forceLocation=forceLocation,
                                    forceMagnitude=forceMagnitude,
                                    inWorldFrame=False)


    def runSingleContactExperiment(self, forceName="iiwa_link_7_1"):
        self.taskRunner.callOnMain(self.externalForce.removeAllForces)
        self.taskRunner.callOnMain(self.addExternalForce, forceName)

        duration = self.config['duration']
        time.sleep(duration)
        self.taskRunner.callOnMain(self.externalForce.removeAllForces)
        # self.externalForce.removeAllForces()

        print "finished running single experiment"

    def runSingleContactExperiment_taskrunner(self, forceName="iiwa_link_7_1"):
        self.taskRunner.callOnThread(self.runSingleContactExperiment, forceName)

    def saveForceLocations(self, filename=None, **kwargs):
        if filename is None:
            filename = "force_locations.out"

        fullFilename = os.path.join(self.spartanSourceDir,"src","ContactParticleFilter", "config", "experiments", filename)

        self.externalForce.saveForceLocationsToFile(fullFilename=fullFilename, **kwargs)