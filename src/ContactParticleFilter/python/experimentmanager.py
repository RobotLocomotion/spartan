import os
import time
import tinydb

from director import ioUtils
from director import lcmUtils
from director import objectmodel as om

import spartan.utils.utils as spartanUtils
from spartan.utils.taskrunner import TaskRunner
import spartan.utils.ros_utils as rosUtils

import utils as cpfUtils
import cpf_lcmtypes


class ExperimentManager(object):

    def __init__(self, robotSystem, robotStateJointController, linkSelection, externalForce, estRobotStatePublisher, configFilename=""):
        self.robotSystem = robotSystem
        self.robotStateJointController = robotStateJointController
        self.linkSelection = linkSelection
        self.externalForce = externalForce
        self.estRobotStatePublisher = estRobotStatePublisher

        self.spartanSourceDir = spartanUtils.getSpartanSourceDir()
        self.cpfSourceDir = cpfUtils.getCPFSourceDir()
        self.loadConfig()
        self.initialize()


        self.initialize()


    def loadConfig(self):
        filename =  os.path.join(self.cpfSourceDir, "config", "experiments", "cpf_experiment_config.yaml")
        self.config = spartanUtils.getDictFromYamlFilename(filename)

        stored_poses_file = os.path.join(self.cpfSourceDir, "config", "experiments", self.config['stored_poses_file'])
        self.storedPosesDict = spartanUtils.getDictFromYamlFilename(stored_poses_file)

    def initialize(self):
        # # start estRobotStatePublisher if we are in sim mode
        # if self.config['mode'] == "simulation":
        #     self.estRobotStatePublisher.start()

        self.loadForcesFromFile()
        self.experimentData = dict()

        self.recordCPFEstimate = False
        self.cpfData = []
        self.taskRunner = TaskRunner()
        self.robotService = rosUtils.RobotService.makeKukaRobotService()
        self.setupSubscribers()

    def isSimulation(self):
        return self.config["mode"] == "simulation"


    def loadForcesFromFile(self, filename=None):
        if filename is None:
            filename = "force_locations.out"

        fullFilename = os.path.join(self.cpfSourceDir, "config", "experiments", filename)

        spartan_source_dir = spartanUtils.getSpartanSourceDir()
        # fullFilename = spartan_source_dir + self.options['data']['initialParticleLocations']
        self.savedForceLocations = ioUtils.readDataFromFile(fullFilename)

    def visualizeForcesFromFile(self):
        self.externalForce.startCaptureMode()
        for key in self.savedForceLocations:
            self.addExternalForce(key)

    def setupSubscribers(self):
        self.subscribers = dict()
        self.msgs = dict()


        self.subscribers['cpf_estimate'] = lcmUtils.addSubscriber("CONTACT_FILTER_POINT_ESTIMATE", cpf_lcmtypes.contact_filter_estimate_t, self.onContactFilterEstimate)
        self.msgs['cpf_estimate'] = []

    def addExternalForce(self, forceName='iiwa_link_7_1'):
        d = self.savedForceLocations[forceName]
        linkName = d['linkName']
        forceLocation = d['forceLocation']
        forceDirection = d['forceDirection']
        # forceMagnitude = d['forceMagnitude']
        forceMagnitude = self.config['force_magnitude']
        self.externalForce.addForceThreadSafe(linkName,
                                    forceDirection=forceDirection,
                                    forceLocation=forceLocation,
                                    forceMagnitude=forceMagnitude,
                                    inWorldFrame=False)


    def runSingleContactExperiment(self, forceName="iiwa_link_7_1", poseName='q_nom', noise_level=0, mode="simulation"):



        # make sure we move to the pose
        print "moving to pose"
        joint_position = self.storedPosesDict[poseName]
        self.robotService.moveToJointPosition(joint_position)
        print "finished moving to pose"


        if mode=="simulation":
            # self.taskRunner.callOnMain(self.externalForce.removeAllForces)
            self.externalForce.removeAllForcesThreadSafe()
            # self.addExternalForce(forceName)
            self.externalForce.options["noise"]["stddev"] = noise_level
            self.externalForce.options["noise"]["addNoise"] = True
            self.addExternalForce(forceName)
            
            # give some time for CPF to reset itself
            time.sleep(2.0)

        
        # if we are in hardware mode then wait here until the CPF starts registering estimates
        if mode=="hardware":
            self.externalForce.stopPublishing()
            self.addExternalForce(forceName) # do this for visualization purposes

            
            while True:
                print "waiting for CPF to detect a contact point"
                if self.lastCPFEstimateMsg.num_contact_points > 0:
                    print "CPF has detected a contact point, starting logging"
                    break
                time.sleep(0.5) # run this at 2Hz

        
        # setup the lcm logger
        lcm_log_file = ExperimentManager.makeUniqueName() + ".lcm"
        lcm_log_full_filename = os.path.join(self.dataFolderName, lcm_log_file)
        lcmLogger = spartanUtils.LCMLogger(lcm_log_full_filename)
        lcmLogger.start()

          

        # also start recording the CPF messages internally
        self.cpfData = [] # clear the previous cache of messages
        self.recordCPFEstimate = True

        # wait for the simulation to run
        duration = self.config['duration']
        time.sleep(duration)
        lcmLogger.stop()
        self.recordCPFEstimate = False


        # record the results and insert into the database
        d = dict()
        d["force_name"] = forceName
        d['pose_name'] = poseName
        d['noise_level'] = noise_level
        d['mode'] = self.config['mode']
        d['lcm_log_file'] = lcm_log_file
        d['force_dict_in_world'] = None
        d['mode'] = self.config['mode']


        # store the data in the database
        self.db.insert(d)

        if mode=="simulation":
            self.externalForce.removeAllForcesThreadSafe()
            self.externalForce.options["noise"]["stddev"] = 0 # set it back to no noise

        # self.taskRunner.callOnMain(self.externalForce.removeAllForces)
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
        unique_name = ExperimentManager.makeUniqueName() + "_" + self.config['mode']
        folderName = os.path.join(self.cpfSourceDir, "data", "experiments", unique_name)
        cmd = "mkdir -p " + folderName
        print cmd
        os.system(cmd)

        time.sleep(0.5)


        self.dataFolderName = folderName
        self.db_json = os.path.join(folderName, 'db.json')
        self.db = tinydb.TinyDB(self.db_json)
        # create tinydb database

    """
    Save the CPF_ESTIMATE messages as soon as they are received`
    """
    def onContactFilterEstimate(self, msg):
        self.lastCPFEstimateMsg = msg
        if self.recordCPFEstimate:
            d = dict()
            # d['ground_truth'] = self.forceDictInWorld
            d['cpf_estimate'] = msg
            self.cpfData.append(d)

            # do something different if we are in hardware mode
            # record the location coming from the OptiTrack system

    # # it's not pretty but it gets the job done
    # def moveToPose_sim(self, poseName="q_nom"):
    #     self.taskRunner.callOnMain(self.estRobotStatePublisher.stop)
        
    #     startPose = self.getPlanningStartPose()
    #     # endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'CPF', 'q_nom')
    #     endPose = self.robotSystem.ikPlanner.getMergedPostureFromDatabase(startPose, 'CPF', poseName) # this is for testing
    #     # ikParameters = IkParameters(maxDegreesPerSecond=60)
    #     plan =self.robotSystem.ikPlanner.computePostureGoal(startPose, endPose)
    #     # self.taskRunner.callOnMain(self.robotSystem.ikPlanner.computePostureGoal, startPose, endPose)
    #     time.sleep(0.5) # wait for the panel to receive the plan
    #     self.taskRunner.callOnMain(self.robotSystem.playbackPanel.executePlan, visOnly=True)
    #     time.sleep(2.0)
    #     # get the plan
    #     # execute the plan visOnly using the playbackPanel
    #     # self.executePlan(visOnly=True)

    #     self.taskRunner.callOnMain(self.estRobotStatePublisher.start)

    def moveToPose(self, poseName='q_nom_down'):
        startPose = self.getPlanningStartPose()
        endPose = self.robotSystem.ikPlanner.getMergedPostureFromDatabase(startPose, 'CPF', poseName)
        numJoints = 7
        endPose = endPose[-numJoints:]
        self.robotService.moveToJointPosition(endPose)
    
    # this should only be called on mainThread
    def moveToPose_sim(self, poseName="q_nom_down"):

        startPose = self.getPlanningStartPose()
        endPose = self.robotSystem.ikPlanner.getMergedPostureFromDatabase(startPose, 'CPF', poseName) 
        plan = self.robotSystem.ikPlanner.computePostureGoal(startPose, endPose)
        self.executePlanSim(plan)
        
    def executePlanSim(self, plan):
        _, poses = self.robotSystem.playbackPanel.planPlayback.getPlanPoses(plan)
        # self.onPlanCommitted(plan)
        self.robotStateJointController.setPose('EST_ROBOT_STATE', poses[-1])

    def moveTest(self):
        joint_position = self.storedPosesDict['above_table_pre_grasp']
        self.robotService.moveToJointPosition(joint_position)

    def moveToPose_test(self, **kwargs):
        self.taskRunner.callOnThread(self.moveTest)


    def runExperimentsSimulation(self):
        self.setupExperimentDataFiles()
        force_names = self.config['force_names']
        
        if len(force_names) == 0:
            force_names= self.savedForceLocations.keys()

        poses = self.config['poses']
        noise_levels = self.config["noise_levels"]

        for poseName in poses:
            for forceName in force_names:
                for noiseLevel in noise_levels:

                    print "\n running single contact experiment, forceName=%(forceName)s, poseName=%(poseName)s, noiseLevel=%(noiseLevel)s", {'forceName':forceName, 'poseName':poseName, 'noiseLevel': noiseLevel}
                    self.runSingleContactExperiment(forceName=forceName, poseName=poseName, noise_level=noiseLevel, mode="simulation")
                    print "finished running single contact experiment \n"


        print "finished running experiments"

    def runExperimentsSimulation_taskrunner(self):
        self.taskRunner.callOnThread(self.runExperimentsSimulation)


    def setupHardwareExperiments(self):
        self.config["mode"] = "hardware"
        self.setupExperimentDataFiles()

        # make the list
        self.experiment_list = []
        for poseName in poses:
            for forceName in force_names:
                self.experiment_list.append((poseName, forceName))

    def runNextHardwareExperiment(self):
        if len(self.experiment_list) == 0:
            print "no more experiments left to run, returning"

        pose_force = self.experiment_list.pop(0)
        print "running experiment for ", pose_force

        self.runSingleContactExperiment(forceName=pose_force[1], poseName=pose_force[0], mode="hardware")


    def runNextHardwareExperiment_taskrunner(self):
        self.taskRunner.callOnThread(self.runNextHardwareExperiment)

    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q


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

    @staticmethod
    def makeUniqueName():
        return time.strftime("%Y%m%d-%H%M%S")


class ForceProbeManager(object):

    """
    optitrackVis is of type OptitrackVisualizer
    """
    def __init__(self, optitrackVis, name="force_probe"):
        self.optitrackVis = optitrackVis
        self.intiialized = False

    """
    Attempt to acquire the "force_probe" vis object from the optitrack visualizer class
    """
    def initialize(self):
        if self.initialized:
            return True

        self.force_probe_vis_obj = om.findObjectByName("force_probe", parent=self.optitrackVis.rigid_bodies)

        if self.force_probe_vis_obj is None:
            return False
        else:
            self.initialized = True
            return True

    def getForceProbeData(self):
        d = dict()
        d['force_location_in_world'] = []
        d['force_direction_in_world'] = []
