import os
import sys
import time
import copy
import imp
import math

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

import tabletop_manipulation_tool
import irb140planning

cameraToWorld = transformUtils.transformFromPose([0, 0, 0], [1, 0, 0, 0])
tableTagToWorld = transformUtils.transformFromPose([0, 0, 0], [1, 0, 0, 0]) 
baseTagToWorld = transformUtils.transformFromPose([0, 0, 0], [1, 0, 0, 0]) 

def setBaseTagToWorld(pos, angle):
    global baseTagToWorld
    if len(angle) == 4:
        rpy = np.array(transformUtils.quaternionToRollPitchYaw(angle))*180./math.pi
    else:
        rpy = angle
    baseTagToWorld = transformUtils.frameFromPositionAndRPY(pos, rpy)
def getBaseTagToWorld():
    return baseTagToWorld
def printbaseTagToWorld():
    print transformUtils.poseFromTransform(getBaseTagToWorld())

setBaseTagToWorld([ 0.10607076,  0.36816069, -0.0355005 ], [ 0.45320318, -0.46124285, -0.53508613,  0.54364027])

def onRobotBaseApriltag(msg, channel):
    global cameraToWorld

    tagToCamera = lcmframe.frameFromRigidTransformMessage(msg)
    vis.updateFrame(tagToCamera, channel, visible=False)

    cameraToTag = tagToCamera.GetLinearInverse()
    baseTagToWorld = getBaseTagToWorld()
    cameraToWorld = transformUtils.concatenateTransforms([cameraToTag, baseTagToWorld])

    cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorld)
    lcmUtils.publish('OPENNI_FRAME_LEFT_TO_LOCAL', cameraToWorldMsg)

    vis.updateFrame(vtk.vtkTransform(), 'world', visible=False)
    vis.updateFrame(cameraToWorld, 'camera to world', visible=False)
    vis.updateFrame(baseTagToWorld, 'base tag to world', visible=False)


def onTabletopApriltag(msg, channel):
    global cameraToWorld, tableTagToWorld
    tagToCamera = lcmframe.frameFromRigidTransformMessage(msg)
    vis.updateFrame(tagToCamera, channel, visible=False)

    tableTagToWorld = transformUtils.concatenateTransforms([tagToCamera, cameraToWorld])

    vis.updateFrame(tableTagToWorld, 'table tag to world', visible=False)


# Robot base apriltag -- #'s 100, 101, 102.
lcmUtils.addSubscriber('APRIL_TAG_0102_TO_CAMERA_LEFT', lcmbotcore.rigid_transform_t, onRobotBaseApriltag, callbackNeedsChannel=True)
# Table surface apriltag -- #50
lcmUtils.addSubscriber('APRIL_TAG_0050_TO_CAMERA_LEFT', lcmbotcore.rigid_transform_t, onTabletopApriltag, callbackNeedsChannel=True)


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

def gripperOpen():
    tabletop_manipulation_tool.sendGripperCommand(0, 125)
def gripperClose():
    tabletop_manipulation_tool.sendGripperCommand(255, 125)
def onFitCamera():
    import aligncameratool
    imp.reload(aligncameratool)
    global alignmentTool
    alignmentTool = aligncameratool.main(robotSystem, newCameraView(imageManager))
def onPlanNominal():
    import irb140planning
    imp.reload(irb140planning)
    irb140planning.planNominalPosture()
def onOpenTaskPanel():
    taskPanel.widget.show()
    taskPanel.widget.raise_()



def setupToolBar():
    toolBar = applogic.findToolBar('Main Toolbar')
    app.addToolBarAction(toolBar, 'Gripper Open', icon='', callback=gripperOpen)
    app.addToolBarAction(toolBar, 'Gripper Close', icon='', callback=gripperClose)
    app.addToolBarAction(toolBar, 'Fit Camera', icon='', callback=onFitCamera)
    app.addToolBarAction(toolBar, 'Plan Nominal', icon='', callback=onPlanNominal)
    app.addToolBarAction(toolBar, 'Open Lonely Panel', icon='', callback=onOpenTaskPanel)



def addToolBarAction(name, callback):
    toolBar = applogic.findToolBar('Main Toolbar')
    app.addToolBarAction(toolBar, name, icon='', callback=callback)


def showLinkFrame(name):
    obj = vis.updateFrame(robotSystem.robotStateModel.getLinkFrame(name), name, parent='link frames')
    obj.setProperty('Scale', 0.2)


def plotPlan():
    robotSystem.planPlayback.plotPlan(robotSystem.manipPlanner.lastManipPlan)

def reloadIrb140Planning():
    import irb140planning
    imp.reload(irb140planning)

def setGripperJointPositions(robotModel, pos):
    robotModel.model.setJointPositions(
        [pos, pos],
        ['wsg_50_finger_left_joint', 'wsg_50_finger_right_joint'])


def havePerceptionDrivers():
    return hasattr(PythonQt.dd, 'ddBotImageQueue')


#####################################################


packageMap = packagepath.PackageMap()
packageMap.populateFromSearchPaths([
    os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'drake', 'drake', 'examples'),
    os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'models')
    ])
roboturdf.addPathsFromPackageMap(packageMap)
print "Package map:"
packageMap.printPackageMap()
robotSystem = makeRobotSystem(view)

import irb140planning
irb140planning.init(robotSystem)

app.addWidgetToDock(robotSystem.teleopPanel.widget, QtCore.Qt.RightDockWidgetArea).hide()
app.addWidgetToDock(robotSystem.playbackPanel.widget, QtCore.Qt.BottomDockWidgetArea).hide()
setupToolBar()

robotLinkSelector = robotlinkselector.RobotLinkSelector()
viewBehaviors.addHandler(viewBehaviors.LEFT_DOUBLE_CLICK_EVENT, robotLinkSelector.onLeftDoubleClick)

if havePerceptionDrivers():
    print "Setting up perception drivers..."
    imageManager = initImageManager()
    openniDepthPointCloud = initDepthPointCloud(imageManager, view)
    cameraView = newCameraView(imageManager)

    frameVisPanel = framevisualization.FrameVisualizationPanel(view)
    app.addWidgetToDock(frameVisPanel.widget, QtCore.Qt.RightDockWidgetArea).hide()

    import loneliest_task_panel
    taskPanel = loneliest_task_panel.LoneliestTaskPanel(robotSystem)
else:
    print "Not setting up perception drivers."

app.restoreDefaultWindowState()
app.initWindowSettings()
applogic.resetCamera(viewDirection=[-1,1,-0.5], view=view)
