import os

from director import visualization
from director import framevisualization
from director import pydrakeik
from director import mainwindowapp
from director import packagepath
from director import robotsystem
from director import roboturdf
from director import drcargs
from director import cameraview
from director import objectmodel as om
from director import segmentation
from director import vtkAll as vtk
from director import affordancemanager
import bot_core as lcmbotcore

import PythonQt
from PythonQt import QtCore, QtGui

import corl.setup
import corl.utils

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

def setupToolBar():
    toolBar = mainwindowapp.applogic.findToolBar('Main Toolbar')
    #fields.app.addToolBarAction(toolBar, 'Gripper Open', icon='', callback=gripperOpen)
    #fields.app.addToolBarAction(toolBar, 'Gripper Close', icon='', callback=gripperClose)
    #fields.app.addToolBarAction(toolBar, 'Task Panel', icon='', callback=onOpenTaskPanel)
    #fields.app.addToolBarAction(toolBar, 'Fit Camera', icon='', callback=onFitCamera)

def showLinkFrame(name):
    obj = visualization.updateFrame(robotSystem.robotStateModel.getLinkFrame(name), name, parent='link frames')
    obj.setProperty('Scale', 0.2)

if __name__ == '__main__':

    # parse args first
    parser = drcargs.getGlobalArgParser().getParser()
    parser.add_argument('--logFolder', type=str, dest='logFolder',
                          help='location of top level folder for this log, relative to CorlDev/data')

    args = parser.parse_args()
    print 'log folder:', args.logFolder

    # construct the app
    fields = mainwindowapp.construct()

    packageMap = packagepath.PackageMap()
    packageMap.populateFromSearchPaths(os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'models'))
    roboturdf.addPathsFromPackageMap(packageMap)

    robotSystem = makeRobotSystem(fields.view)

    fields.app.addWidgetToDock(robotSystem.teleopPanel.widget, QtCore.Qt.RightDockWidgetArea).hide()
    fields.app.addWidgetToDock(robotSystem.playbackPanel.widget, QtCore.Qt.BottomDockWidgetArea).hide()
    setupToolBar()

    # show the main window and start the app
    fields.app.start()
