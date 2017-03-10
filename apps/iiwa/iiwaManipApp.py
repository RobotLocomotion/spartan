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
from director import fieldcontainer
from director import framevisualization
from director import drcargs


# todo-
# the optitrack driver build system needs to lcm gen and install python files
# for now, set PYTHONPATH manually in order to find and import the optitrack lcmtypes
sys.path.append(os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'drivers/optitrack'))
from director.optitrackvisualizer import OptitrackVisualizer

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


#####################################################


packageMap = packagepath.PackageMap()
packageMap.populateFromSearchPaths(os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'models'))
roboturdf.addPathsFromPackageMap(packageMap)

robotSystem = makeRobotSystem(view)


optitrackVis = OptitrackVisualizer('OPTITRACK_FRAMES')
optitrackVis.setEnabled(True)
applogic.MenuActionToggleHelper('Tools', optitrackVis.name, optitrackVis.isEnabled, optitrackVis.setEnabled)

app.addWidgetToDock(robotSystem.teleopPanel.widget, QtCore.Qt.RightDockWidgetArea).hide()
app.addWidgetToDock(robotSystem.playbackPanel.widget, QtCore.Qt.BottomDockWidgetArea).hide()
setupToolBar()


setGripperJointPositions(robotSystem.robotStateModel, 0.04)
setGripperJointPositions(robotSystem.teleopRobotModel, 0.04)
setGripperJointPositions(robotSystem.playbackRobotModel, 0.04)

ikPlanner = robotSystem.ikPlanner
robotStateJointController = robotSystem.robotStateJointController

def getPlanningStartPose():
    return robotStateJointController.q.copy()


def computeBimanualPosture(leftPostureName, rightPostureName):
    endPose = getPlanningStartPose()
    endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'General', leftPostureName, 'left')
    endPose = ikPlanner.getMergedPostureFromDatabase(endPose, 'General', rightPostureName, 'right')
    return endPose

def planBimanualPosture(leftPostureName, rightPostureName, startPose=None):
    if startPose is None:
        startPose = getPlanningStartPose()
    endPose = computeBimanualPosture(leftPostureName, rightPostureName)
    return ikPlanner.computePostureGoal(startPose, endPose)

def planBothZero():
    planBimanualPosture('q_zero', 'q_zero')

def planBothNominal():
    planBimanualPosture('q_nom', 'q_nom')

def planLeftZeroRightNominal():
    planBimanualPosture('q_zero', 'q_nom')

def planLeftNominalRightZero():
    planBimanualPosture('q_nom', 'q_zero')

def bimanualDemo():
    from director import pydrakeik
    pydrakeik.useWarpTime = False
    poses = [getPlanningStartPose(),
             computeBimanualPosture('q_nom', 'q_nom'),
             computeBimanualPosture('q_zero', 'q_zero'),
             computeBimanualPosture('q_nom', 'q_nom'),
             computeBimanualPosture('q_zero', 'q_nom'),
             computeBimanualPosture('q_nom', 'q_zero'),
             computeBimanualPosture('q_zero', 'q_nom'),
             computeBimanualPosture('q_nom', 'q_zero'),
             computeBimanualPosture('q_nom', 'q_nom')]
    plans = []
    for a, b in zip(poses, poses[1:]):
        ikPlanner.computePostureGoal(a, b)
        plans.append(ikPlanner.lastManipPlan)
    plan = robotSystem.planPlayback.mergePlanMessages(plans)
    lcmUtils.publish('CANDIDATE_MANIP_PLAN', plan)


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


app.restoreDefaultWindowState()
app.initWindowSettings()
applogic.resetCamera(viewDirection=[-1,1,-0.5], view=view)
