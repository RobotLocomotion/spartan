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
from director import packagepath
from director import roboturdf
from director import fieldcontainer
from director import framevisualization
from director import drcargs


import PythonQt
from PythonQt import QtCore, QtGui


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

#setupKinect()


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
    factory.setDependentOptions(options,useSegmentation=True, useSegmentationAffordances=True)
    robotSystem = factory.construct(view=view, options=options)

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
    #app.addToolBarAction(toolBar, 'Gripper Open', icon='', callback=gripperOpen)
    #app.addToolBarAction(toolBar, 'Gripper Close', icon='', callback=gripperClose)
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


def makeDrakeSimObjectSync(drakeSimName, affordanceName):

    def onTimerCallback():
        simObj = om.findObjectByName(drakeSimName)
        if not simObj or not simObj.actor.GetUserTransform():
            return

        obj = om.findObjectByName(affordanceName)
        obj.getChildFrame().copyFrame(simObj.actor.GetUserTransform())

    t = TimerCallback(callback=onTimerCallback)
    return t

####################################################



def mySave():
  foo()



#####################################################




addToolBarAction('Save Data', mySave)

robotSystem = makeRobotSystem(view)

imageManager = initImageManager()
openniDepthPointCloud = initDepthPointCloud(imageManager, view)
cameraView = newCameraView(imageManager)


