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

import PythonQt
from PythonQt import QtCore, QtGui

import drake as lcmdrake
import bot_core as lcmbotcore

def initImageManager():
    imageManager = cameraview.ImageManager()
    cameraview.imageManager = imageManager
    return imageManager
    
def initDepthPointCloud(imageManager, view):

    openniDepthPointCloud = segmentation.DisparityPointCloudItem('gelsight point cloud', 'GELSIGHT_RECONSTRUCTION', 'OPENNI_FRAME_LEFT', imageManager)
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


def havePerceptionDrivers():
    return hasattr(PythonQt.dd, 'ddBotImageQueue')


#####################################################


if havePerceptionDrivers():

    import mytaskpanel

    imageManager = initImageManager()
    openniDepthPointCloud = initDepthPointCloud(imageManager, view)
    cameraView = newCameraView(imageManager)

else:
    print "Have no perception drivers"

app.restoreDefaultWindowState()
app.initWindowSettings()
applogic.resetCamera(viewDirection=[-1,1,-0.5], view=view)
