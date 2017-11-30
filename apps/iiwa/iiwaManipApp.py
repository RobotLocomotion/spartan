import os
import sys
import time
import copy
import imp

from director import lcmUtils
from director import lcmframe
from director import transformUtils
from director import robotsystem
from director import segmentation
from director import cameraview
from director import pydrakeik
from director import packagepath
from director import roboturdf
from director import robotlinkselector
from director import fieldcontainer
from director import framevisualization
from director import drcargs


try:
    from director.optitrackvisualizer import OptitrackVisualizer
    useOptitrackVisualizer = True

except ImportError:
    # optitrack lcmtypes are not available
    useOptitrackVisualizer = False


import PythonQt
from PythonQt import QtCore, QtGui


import drake as lcmdrake
import bot_core as lcmbotcore

from spartan.manipulation.director_schunk_driver import DirectorSchunkDriver


def setTagToWorld(pos, rpy):
    global tagToWorld
    tagToWorld = transformUtils.frameFromPositionAndRPY(pos, rpy)


def getTagToWorld():
    return tagToWorld


setTagToWorld([0.29,-0.38,-0.10], [-93,1,1.5])


def onAprilTagMessage(msg, channel):
    tagToCamera = lcmframe.frameFromRigidTransformMessage(msg)
    vis.updateFrame(tagToCamera, channel, visible=False)

    cameraToTag = tagToCamera.GetLinearInverse()
    tagToWorld = getTagToWorld()
    cameraToWorld = transformUtils.concatenateTransforms([cameraToTag, tagToWorld])

    cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorld)
    lcmUtils.publish('OPENNI_FRAME_LEFT_TO_LOCAL', cameraToWorldMsg)

    vis.updateFrame(vtk.vtkTransform(), 'world', visible=False)
    vis.updateFrame(cameraToWorld, 'camera to world', visible=False)
    vis.updateFrame(tagToWorld, 'tag to world', visible=False)


lcmUtils.addSubscriber('APRIL_TAG_0218_TO_CAMERA_LEFT', lcmbotcore.rigid_transform_t, onAprilTagMessage, callbackNeedsChannel=True)


def setupKinect():
    from director import kinectlcm
    kinectlcm.init(view)


def setCameraToWorld(cameraToWorld):
    cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorld)
    lcmUtils.publish('OPENNI_FRAME_LEFT_TO_LOCAL', cameraToWorldMsg)


def getCameraToWorld():
    cameraName = 'OPENNI_FRAME_LEFT'
    q = cameraview.imageManager.queue
    utime = q.getCurrentImageTime(cameraName)
    t = vtk.vtkTransform()
    q.getTransform(cameraName, 'local', utime, t)
    return t


def setTfRootForCamera():

    tfVis.setTfToWorld(vtk.vtkTransform())

    opticalToRoot = transformUtils.copyFrame(om.findObjectByName('camera_depth_optical_frame').transform)
    rootToOptical = opticalToRoot.GetLinearInverse()
    cameraToWorld = getCameraToWorld()
    t = transformUtils.concatenateTransforms([rootToOptical, cameraToWorld])
    tfVis.setTfToWorld(t)


def makeRobotSystem(view):

    factory = robotsystem.ComponentFactory()
    factory.register(robotsystem.RobotSystemFactory)
    options = factory.getDisabledOptions()
    factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True, useSegmentation=True, useSegmentationAffordances=True)
    robotSystem = factory.construct(view=view, options=options)

    # use pydrake ik backend
    ikPlanner = robotSystem.ikPlanner
    ikPlanner.planningMode = 'pydrake'
    ikPlanner.plannerPub._setupLocalServer()

    # set default options
    #robotSystem.playbackPanel.animateOnExecute = True
    ikPlanner.addPostureGoalListener(robotSystem.robotStateJointController)
    ikPlanner.getIkOptions().setProperty('Max joint degrees/s', 60)
    ikPlanner.getIkOptions().setProperty('Use pointwise', False)

    return robotSystem


def initImageManager():
    imageManager = cameraview.ImageManager()
    cameraview.imageManager = imageManager
    return imageManager


def initDepthPointCloud(imageManager, view):

    openniDepthPointCloud = segmentation.DisparityPointCloudItem('openni point cloud', 'OPENNI_FRAME', 'OPENNI_FRAME_LEFT', imageManager)
    openniDepthPointCloud.addToView(view)
    om.addToObjectModel(openniDepthPointCloud, parentObj=om.findObjectByName('sensors'))
    openniDepthPointCloud.setProperty('Visible', True)
    openniDepthPointCloud.setProperty('Target FPS', 30)
    return openniDepthPointCloud


def newCameraView(imageManager, channelName='OPENNI_FRAME', cameraName='OPENNI_FRAME_LEFT', viewName='OpenNI Frame'):

    view = PythonQt.dd.ddQVTKWidgetView()
    view.orientationMarkerWidget().Off()
    view.backgroundRenderer().SetBackground([0,0,0])
    view.backgroundRenderer().SetBackground2([0,0,0])

    imageManager.queue.addCameraStream(channelName, cameraName, lcmbotcore.images_t.LEFT)
    imageManager.addImage(cameraName)

    cameraView = cameraview.CameraImageView(imageManager, cameraName, viewName=viewName, view=view)
    cameraView.eventFilterEnabled = False
    view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleImage())

    return cameraView


def sendGripperCommand(targetPositionMM, force):
    msg = lcmdrake.lcmt_schunk_wsg_command()
    msg.utime = int(time.time()*1e6)
    msg.force = force
    msg.target_position_mm = targetPositionMM
    lcmUtils.publish('SCHUNK_WSG_COMMAND', msg)


def gripperOpen():
    sendGripperCommand(100, 40)


def gripperClose():
    sendGripperCommand(0, 40)



schunkDriver = DirectorSchunkDriver()


def onOpenTaskPanel():
    taskPanel.widget.show()
    taskPanel.widget.raise_()


def onFitCamera():
    import aligncameratool
    imp.reload(aligncameratool)
    global alignmentTool
    alignmentTool = aligncameratool.main(robotSystem, newCameraView(imageManager))


def setupToolBar():
    toolBar = applogic.findToolBar('Main Toolbar')
    app.addToolBarAction(toolBar, 'Gripper Open', icon='', callback=schunkDriver.sendOpenGripperCommand)
    app.addToolBarAction(toolBar, 'Gripper Close', icon='', callback=schunkDriver.sendCloseGripperCommand)
    app.addToolBarAction(toolBar, 'Task Panel', icon='', callback=onOpenTaskPanel)
    app.addToolBarAction(toolBar, 'Fit Camera', icon='', callback=onFitCamera)


def addToolBarAction(name, callback):
    toolBar = applogic.findToolBar('Main Toolbar')
    app.addToolBarAction(toolBar, name, icon='', callback=callback)


def showLinkFrame(name):
    obj = vis.updateFrame(robotSystem.robotStateModel.getLinkFrame(name), name, parent='link frames')
    obj.setProperty('Scale', 0.2)


def plotPlan():
    robotSystem.planPlayback.plotPlan(robotSystem.manipPlanner.lastManipPlan)


def setGripperJointPositions(robotModel, pos):
    robotModel.model.setJointPositions(
        [pos, pos],
        ['wsg_50_base_joint_gripper_left', 'wsg_50_base_joint_gripper_right'])


def reloadIiwaPlanning():
    imp.reload(ip)


def havePerceptionDrivers():
    return hasattr(PythonQt.dd, 'ddBotImageQueue')


def makeDrakeSimObjectSync(drakeSimName, affordanceName):

    def onTimerCallback():
        simObj = om.findObjectByName(drakeSimName)
        if not simObj or not simObj.actor.GetUserTransform():
            return

        obj = om.findObjectByName(affordanceName)
        obj.getChildFrame().copyFrame(simObj.actor.GetUserTransform())

    t = TimerCallback(callback=onTimerCallback)
    return t


# todo
# write a OPTITRACK_TO_FRAMES bot-frames translator
# add bot-frames to iiwaManip.cfg file for robot_to_optitrack_body, and optitrack_to_world.
# state translator uses bot-frames to set floating base joint in EST_ROBOT_STATE
def setRobotPoseFromOptitrack(robotToBodyOffset=[0.0, -0.035, 0.0, -90, -90.5, 0], optitrackBodyName='Body 1'):
    bodyToWorld = om.findObjectByName(optitrackBodyName + ' frame').transform
    robotToBody = transformUtils.frameFromPositionAndRPY(robotToBodyOffset[:3], robotToBodyOffset[3:])
    robotToWorld = transformUtils.concatenateTransforms([robotToBody, bodyToWorld])
    pos, quat = transformUtils.poseFromTransform(robotToWorld)
    rpy = transformUtils.quaternionToRollPitchYaw(quat)
    robotSystem.robotStateJointController.q[:6] = np.hstack((pos,rpy))
    robotSystem.robotStateJointController.push()


#####################################################


packageMap = packagepath.PackageMap()
packageMap.populateFromSearchPaths(os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'models'))
roboturdf.addPathsFromPackageMap(packageMap)

robotSystem = makeRobotSystem(view)


# TODO: move this to director/robotsystem.py as optional feature
if useOptitrackVisualizer:
    optitrackVis = OptitrackVisualizer('OPTITRACK_FRAMES')
    optitrackVis.setEnabled(True)
    applogic.MenuActionToggleHelper('Tools', optitrackVis.name, optitrackVis.isEnabled, optitrackVis.setEnabled)


app.addWidgetToDock(robotSystem.teleopPanel.widget, QtCore.Qt.RightDockWidgetArea).hide()
app.addWidgetToDock(robotSystem.playbackPanel.widget, QtCore.Qt.BottomDockWidgetArea).hide()
setupToolBar()


setGripperJointPositions(robotSystem.robotStateModel, 0.04)
setGripperJointPositions(robotSystem.teleopRobotModel, 0.04)
setGripperJointPositions(robotSystem.playbackRobotModel, 0.04)


robotLinkSelector = robotlinkselector.RobotLinkSelector()
viewBehaviors.addHandler(viewBehaviors.LEFT_DOUBLE_CLICK_EVENT, robotLinkSelector.onLeftDoubleClick)




if havePerceptionDrivers():

    

    imageManager = initImageManager()
    openniDepthPointCloud = initDepthPointCloud(imageManager, view)
    cameraView = newCameraView(imageManager)

    frameVisPanel = framevisualization.FrameVisualizationPanel(view)
    app.addWidgetToDock(frameVisPanel.widget, QtCore.Qt.RightDockWidgetArea).hide()

    try:
        import mytaskpanel
        taskPanel = mytaskpanel.MyTaskPanel(robotSystem, cameraView)
        taskPanel.planner.openGripperFunc = gripperOpen
        taskPanel.planner.closeGripperFunc = gripperClose

        ip = mytaskpanel.iiwaplanning
    except:
        print "couldn't find mytaskpanel, skipping"

    #affordanceName = 'box'
    #ip.spawnAffordance(affordanceName)
    #ip.addGraspFrames(affordanceName)
    #ip.makeBestPlan(affordanceName)
    #addToolBarAction('Random test', ip.randomTest)


    #syncTimer = makeDrakeSimObjectSync('blue_funnel geometry data', 'blue funnel')
    #syncTimer.start()


useCorlDev = False
if useCorlDev:

    assert havePerceptionDrivers()

    # these lines are used to setup the camera pose updater based on robot arm kinematics
    # not used anymore, just a test.  doesn't work that well because the camera to end-effector
    # pose is estimated roughly and not properly calibrated.
    #initRobotKinematicsCameraFrame()
    #robotSystem.robotStateModel.connectModelChanged(updateCameraPoseFromRobotKinematics)

    import corl.setup
    import corl.imagecapture
    # extraInputs = dict()
    # extraInputs['imageManager'] = imageManager
    # globals()['imageManager'] = imageManager
    # corl.setup.setupCorlDirector(robotSystem.affordanceManager,
    #                              openniDepthPointCloud,
    #                              logFolder="logs/moving-camera",
    #                              globalsDict=globals())

    corl.setup.setupDataCollection(globals())

app.restoreDefaultWindowState()
app.initWindowSettings()
applogic.resetCamera(viewDirection=[-1,1,-0.5], view=view)


useKukaRLGDev = True
if useKukaRLGDev:

    # broadcast the pose of the wrist mounted Xtion
    import spartan.utils.utils as spartanUtils

    camera_serial_number = 1112170110
    calibration_folder = 'master'

    cameraConfigFilename = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects', 'camera_config', 'data', str(camera_serial_number), calibration_folder, 'camera_info.yaml')

    channelName = "OPENNI_FRAME_LEFT"

    import spartan.perception.cameratransform
    cameraTransform = spartan.perception.cameratransform.CameraTransform.fromConfigFilename(robotSystem, cameraConfigFilename, channelName)

    # import spartan.perception.dev as devUtils
    # efusion = devUtils.ElasticFusionReconstruction()

    import spartan.calibration.handeyecalibration
    cal = spartan.calibration.handeyecalibration.HandEyeCalibration(robotSystem)

    import spartan.director.iiwamanipdev
    spartan.director.iiwamanipdev.setupRLGDirector(globals())

    # setup the director node

useROS = True
if useROS:
    import rospy
    rospy.init_node('director')