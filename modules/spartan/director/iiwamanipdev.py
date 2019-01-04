#system
import os
import time 

# director
import director.objectmodel as om
import director.visualization as vis
from director import ioUtils

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

    # set rate limit on RemoteTreeViewer
    # fix for https://github.com/RobotLocomotion/spartan/issues/244
    globalsDict['treeViewer'].subscriber.setSpeedLimit(5)


    # load background scene if it exists
    background_ply_file = os.path.join(spartanUtils.get_data_dir(), 'pdc', 'logs_special',
        '2019-01-03-22-43-55', 'processed', 'fusion_mesh.ply')

    def visualize_background():
        if not os.path.exists(background_ply_file):
            return


        parent = om.getOrCreateContainer("scene")
        poly_data = ioUtils.readPolyData(background_ply_file)
        vis.updatePolyData(poly_data, 'table', parent=parent)


    visualize_background()




