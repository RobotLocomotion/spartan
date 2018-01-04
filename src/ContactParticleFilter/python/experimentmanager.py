import os
import time
import tinydb

from director import ioUtils

import spartan.utils.utils as spartanUtils
from spartan.utils.taskrunner import TaskRunner

import utils as cpfUtils

class ExperimentManager(object):

    def __init__(self, robotStateModel, robotStateJointController, linkSelection, externalForce, estRobotStatePublisher, configFilename=""):
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.linkSelection = linkSelection
        self.externalForce = externalForce
        self.estRobotStatePublisher = estRobotStatePublisher
        self.spartanSourceDir = spartanUtils.getSpartanSourceDir()
        self.cpfSourceDir = cpfUtils.getCPFSourceDir()
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

        fullFilename = os.path.join(self.cpfSourceDir, "config", "experiments", filename)

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


    def runSingleContactExperiment(self, forceName="iiwa_link_7_1", poseName='q_nom', noise_level=0):
        self.taskRunner.callOnMain(self.externalForce.removeAllForces)
        self.taskRunner.callOnMain(self.addExternalForce, forceName)

        duration = self.config['duration']
        time.sleep(duration)

        # insert into the database
        d = dict()
        d["force_name"] = forceName
        d['pose_name'] = poseName
        d['noise_level'] = noise_level
        d['mode'] = self.config['mode']
        d['stats'] = None # fill this in later
        d['lcm_log_file'] = None


        self.db.insert(d)

        self.taskRunner.callOnMain(self.externalForce.removeAllForces)
        # self.externalForce.removeAllForces()

        print "finished running single experiment"

    def runSingleContactExperiment_taskrunner(self, forceName="iiwa_link_7_1"):
        self.taskRunner.callOnThread(self.runSingleContactExperiment, forceName)

    def saveForceLocations(self, filename=None, **kwargs):
        if filename is None:
            filename = "force_locations.out"

        fullFilename = os.path.join(self.cpfSourceDir, "config", "experiments", filename)

        self.externalForce.saveForceLocationsToFile(fullFilename=fullFilename, **kwargs)


    def setupExperimentDataFiles(self):
        unique_name = time.strftime("%Y%m%d-%H%M%S") + "_" + self.config['mode']
        folderName = os.path.join(self.cpfSourceDir, "data", "experiments", unique_name)
        cmd = "mkdir -p " + folderName
        print cmd
        os.system(cmd)

        time.sleep(0.5)


        self.dataFolderName = folderName
        self.db_json = os.path.join(folderName, 'db.json')
        self.db = tinydb.TinyDB(self.db_json)
        # create tinydb database


    def saveExperimentData(self):
        pass

    ##### testing
    def test(self):
        self.setupExperimentDataFiles()
        self.runSingleContactExperiment_taskrunner()
        self.saveExperimentData()

    def testLogger(self):
        filename = os.path.join(self.spartanSourceDir, 'sandbox', "log_test.lcm")
        self.logger = spartanUtils.LCMLogger(filename)
        self.logger.start()
