#system
import os
import time 

# director
from director.debugVis import DebugData
import director.visualization as vis

# spartan
import spartan.manipulation.grasp_supervisor
import spartan.manipulation.background_subtraction
import spartan.calibration.handeyecalibration
import spartan.utils.utils as spartanUtils


from spartan.utils.taskrunner import TaskRunner

# ros
import tf2_ros


class TFWrapper(object):

    def __init__(self):
        self.tfBuffer = None
        self.tfListener = None
        self.taskRunner = TaskRunner()
        self.taskRunner.callOnThread(self.setup)

    def setup(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def getBuffer(self):
        while self.tfBuffer is None:
            time.sleep(0.1)

        return self.tfBuffer


def setupRLGDirector(globalsDict=None):

    tfWrapper = TFWrapper()
    tfBuffer = tfWrapper.getBuffer()

    graspSupervisor = spartan.manipulation.grasp_supervisor.GraspSupervisor.makeDefault(tfBuffer=tfBuffer)
    graspSupervisor.robotSystem = globalsDict['robotSystem'] # for visualization
    globalsDict['graspSupervisor'] = graspSupervisor

    
    backgroundSubtraction = spartan.manipulation.background_subtraction.BackgroundSubtractionDataCapture.makeDefault(tfBuffer=tfBuffer)
    globalsDict['backgroundSubtraction'] = backgroundSubtraction



    spartanSourceDir = spartanUtils.getSpartanSourceDir()
    handEyeCalibrationConfigFilename = os.path.join(spartanSourceDir, "src/catkin_projects/station_config/RLG_iiwa_1/hand_eye_calibration/carmine_1.yaml")


    cal = spartan.calibration.handeyecalibration.HandEyeCalibration(globalsDict['robotSystem'], configFilename=handEyeCalibrationConfigFilename)
    cal.loadConfigFromFile()
    globalsDict['cal'] = cal

    # add sphere
    d = DebugData()
    center = [0.61, 0.15, 0.0]
    d.addSphere(center, radius=0.015, color=[0,1,0])
    vis.showPolyData(d.getPolyData(), "goal position",  color=[0,1,0])

    # add line
    # # add sphere
    # d = DebugData()
    # y_pos = 0.176
    # start = [0.39, y_pos, 0.005]
    # end = [1.09, y_pos, 0.005]
    # d.addLine(start, end, radius=0.01, color=[0, 1, 0])
    # vis.showPolyData(d.getPolyData(), "goal line", color=[0, 1, 0])

    # set rate limit on RemoteTreeViewer
    # fix for https://github.com/RobotLocomotion/spartan/issues/244
    globalsDict['treeViewer'].subscriber.setSpeedLimit(5)