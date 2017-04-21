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
    app.addToolBarAction(toolBar, 'Gripper Open', icon='', callback=gripperOpen)
    app.addToolBarAction(toolBar, 'Gripper Close', icon='', callback=gripperClose)
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
        ['wsg_50_finger_left_joint', 'wsg_50_finger_right_joint'])


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


def initRobotKinematicsCameraFrame():
    endEffectorToWorld = robotSystem.robotStateModel.getLinkFrame('iiwa_link_ee')
    frameObj = vis.updateFrame(endEffectorToWorld, 'iiwa_link_ee', parent='debug', scale=0.15, visible=False)
    cameraToEE = transformUtils.frameFromPositionAndRPY([0.1,0,0.0], [-90,-22.5,-90])
    cameraToWorld = transformUtils.concatenateTransforms([cameraToEE, endEffectorToWorld])
    obj = vis.updateFrame(cameraToWorld, 'camera frame', parent=frameObj, scale=0.15)
    frameObj.getFrameSync().addFrame(obj, ignoreIncoming=True)

    def onCameraFrameModified(f):
        setCameraToWorld(f.transform)

    obj.connectFrameModified(onCameraFrameModified)


def updateCameraPoseFromRobotKinematics(model):
    endEffectorToWorld = model.getLinkFrame('iiwa_link_ee')
    vis.updateFrame(endEffectorToWorld, 'iiwa_link_ee', parent='debug', scale=0.15, visible=False)


def getDefaultCameraToWorld():
    return transformUtils.frameFromPositionAndRPY([0,0,0], [-90,0,-90])


def loadPhoneModel():
    filename = os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'src/CorlDev/data/object-meshes/phone.vtp')
    name = 'phone'
    affordanceManager = robotSystem.affordanceManager
    pose = ([ 0.76200759, -0.03861831, -0.24704412],
            [-0.37927565, -0.39252409, -0.16826478,  0.82082994])

    return affordanceManager.newAffordanceFromDescription(
          dict(classname='MeshAffordanceItem', Name=name,
               pose=pose,
               Filename=filename))


def loadElasticFustionReconstruction():
    filename = os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'src/CorlDev/data/efusion-output/moving-camera/moving-camera.lcmlog.vtp')
    polyData = ioUtils.readPolyData(filename)
    polyData = filterUtils.transformPolyData(polyData, getDefaultCameraToWorld())
    obj = vis.showPolyData(polyData, 'reconstruction', colorByName='RGB')
    return obj


def initCameraUpdateCallback(obj):

    f = os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'src/CorlDev/data/efusion-output/moving-camera/moving-camera.lcmlog.posegraph')
    data = np.loadtxt(f)
    poseTimes = np.array(data[:,0]*1e6, dtype=int)
    poses = np.array(data[:,1:])

    def getCameraPoseAtTime(t):
        ind = poseTimes.searchsorted(t)
        if ind == len(poseTimes):
            ind = len(poseTimes)-1
        pose = poses[ind]
        pos = pose[:3]
        quat = pose[6], pose[3], pose[4], pose[5] # quat data from file is ordered as x, y, z, w
        return transformUtils.transformFromPose(pos, quat)

    def myUpdate():
        lastUtime = obj.lastUtime
        obj.update()
        if obj.lastUtime == lastUtime:
            return


        cameraToCameraStart = getCameraPoseAtTime(obj.lastUtime)
        t = transformUtils.concatenateTransforms([cameraToCameraStart, getDefaultCameraToWorld()])

        vis.updateFrame(t, 'camera pose')

        useAffordanceProjection = True

        if useAffordanceProjection:
            setCameraToWorld(t)
        else:
            obj.actor.SetUserTransform(t)

    obj.timer.callback = myUpdate



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

    import mytaskpanel

    imageManager = initImageManager()
    openniDepthPointCloud = initDepthPointCloud(imageManager, view)
    cameraView = newCameraView(imageManager)

    frameVisPanel = framevisualization.FrameVisualizationPanel(view)
    app.addWidgetToDock(frameVisPanel.widget, QtCore.Qt.RightDockWidgetArea).hide()

    taskPanel = mytaskpanel.MyTaskPanel(robotSystem, cameraView)
    taskPanel.planner.openGripperFunc = gripperOpen
    taskPanel.planner.closeGripperFunc = gripperClose

    ip = mytaskpanel.iiwaplanning

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

    initCameraUpdateCallback(openniDepthPointCloud)
    loadPhoneModel()
    loadElasticFustionReconstruction()


app.restoreDefaultWindowState()
app.initWindowSettings()
applogic.resetCamera(viewDirection=[-1,1,-0.5], view=view)
