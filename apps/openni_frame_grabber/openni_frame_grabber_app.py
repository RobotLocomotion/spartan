import os
import sys
import time
import copy
import imp
import math

from director import lcmUtils
from director import lcmframe
from director import transformUtils
from director import segmentation
from director import cameraview
from director import pydrakeik
from director import packagepath
from director import roboturdf
from director import robotlinkselector
from director import fieldcontainer
from director import drcargs

import PythonQt
from PythonQt import QtCore, QtGui


import drake as lcmdrake
import bot_core as lcmbotcore

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

def onSaveOpenniFrame():
    obj = om.findObjectByName('openni point cloud')
    pd = obj.polyData

    import datetime
    prefix = datetime.datetime.now().isoformat()
    print "Saving %s...vtp" % prefix
    ioUtils.writePolyData(pd, prefix + "_original.vtp")
    
def setupToolBar():
    toolBar = applogic.findToolBar('Main Toolbar')
    app.addToolBarAction(toolBar, 'Save Openni Frame', icon='', callback=onSaveOpenniFrame)


def addToolBarAction(name, callback):
    toolBar = applogic.findToolBar('Main Toolbar')
    app.addToolBarAction(toolBar, name, icon='', callback=callback)

def havePerceptionDrivers():
    return hasattr(PythonQt.dd, 'ddBotImageQueue')


#####################################################

setupToolBar()

if havePerceptionDrivers():
    print "Setting up perception drivers..."
    imageManager = initImageManager()
    openniDepthPointCloud = initDepthPointCloud(imageManager, view)
    cameraView = newCameraView(imageManager)

else:
    print "Not setting up perception drivers."

app.restoreDefaultWindowState()
app.initWindowSettings()
applogic.resetCamera(viewDirection=[-1,1,-0.5], view=view)
