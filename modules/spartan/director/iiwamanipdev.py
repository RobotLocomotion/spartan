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
import spartan.utils.director_utils as director_utils
from spartan.manipulation.object_manipulation import ObjectManipulation
from spartan.poser.poser_visualizer import PoserVisualizer
from spartan.utils.taskrunner import TaskRunner
from spartan.manipulation.category_manipulation import CategoryManipulation
from spartan.utils.director_ros_visualizer import DirectorROSVisualizer

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


    ros_visualizer = DirectorROSVisualizer(tf_buffer=tfBuffer)
    topic = "/camera_carmine_1/depth/points"
    ros_visualizer.add_subscriber(topic, name="Carmine", visualize=True)
    globalsDict['ros_visualizer'] = ros_visualizer
    ros_visualizer.start()


    # load background scene if it exists
    background_ply_file = os.path.join(spartanUtils.get_data_dir(), 'pdc', 'logs_special',
        '2019-01-03-22-43-55', 'processed', 'fusion_mesh.ply')


    robotSystem = globalsDict['robotSystem']
    robotStateModel = robotSystem.robotStateModel

    category_manip = CategoryManipulation(robotStateModel)
    category_manip.load_side_table()
    # category_manip.load_mug_rack()
    # category_manip.setup_horizontal_mug_grasp()
    # category_manip.load_mug_platform()
    category_manip.load_shoe_rack()

    def visualize_background():
        if not os.path.exists(background_ply_file):
            return


        parent = om.getOrCreateContainer("scene")
        poly_data = ioUtils.readPolyData(background_ply_file)
        vis.updatePolyData(poly_data, 'table', parent=parent)


    visualize_background()

    poser_vis = PoserVisualizer.make_default()
    object_manip = ObjectManipulation()
    globalsDict['object_manip'] = object_manip
    globalsDict['o'] = object_manip
    globalsDict['c'] = category_manip
    globalsDict['poser_vis'] = poser_vis
    globalsDict['g'] = graspSupervisor




