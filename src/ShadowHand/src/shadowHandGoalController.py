import os
import sys
import PythonQt
import time
from PythonQt import QtCore, QtGui

from director import mainwindowapp
from director import treeviewer
from director import jointcontrol
from director import lcmUtils
from director import robotstate
from director import roboturdf
from director import transformUtils
from director import visualization as vis
from director.timercallback import TimerCallback
from director import objectmodel as om
from director.debugVis import DebugData
import functools
import numpy as np

from drake import lcmt_body_motion_data, lcmt_polynomial_matrix, lcmt_polynomial

class ControlTargetControlWidget(object):

    def __init__(self, view, robotModel):
        self.view = view
        self.robotModel = robotModel
        self.selectedLink = None

        targetTF = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.3], [0., 0., 0.])
        self.targetFrame = vis.showFrame(targetTF, "Goal Frame", parent="Shadow Hand control", scale=0.05)
        self.targetFrame.connectFrameModified(self.onGoalFrameModified)
            
        self.lastMousePress = time.time()

    def onGoalFrameModified(self, frame):
        if self.selectedLink:
            tf = transformUtils.getNumpyFromTransform(self.targetFrame.transform)
            msg = lcmt_body_motion_data()
            msg.body_or_frame_name = self.selectedLink
            msg.weight_multiplier[0] = 500

            msg.spline.num_breaks = 2
            msg.spline.breaks = [0, 9999999999999]
            msg.spline.num_segments = 1
            msg.spline.polynomial_matrices = [lcmt_polynomial_matrix()]
            msg.spline.polynomial_matrices[0].rows = 3
            msg.spline.polynomial_matrices[0].cols = 1
            for i in range(3):
                msg.spline.polynomial_matrices[0].polynomials.append([lcmt_polynomial()])
                msg.spline.polynomial_matrices[0].polynomials[i][0].num_coefficients = 1
                msg.spline.polynomial_matrices[0].polynomials[i][0].coefficients = [tf[i, 3]]

            print "Submitting goal for ", msg.body_or_frame_name
            lcmUtils.publish("SH_GOALS", msg)

    def start(self):
        self.installEventFilter()

    def stop(self):
        self.removeEventFilter()

    def installEventFilter(self):

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.view.vtkWidget().installEventFilter(self.eventFilter)

        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

    def removeEventFilter(self):
        self.view.vtkWidget().removeEventFilter(self.eventFilter)

    def onEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseButtonDblClick:
            self.eventFilter.setEventHandlerResult(
                self.onDoubleClick(vis.mapMousePosition(obj, event), event.modifiers())
            )

    def getSelection(self, displayPoint):
        ret = vis.pickPoint(displayPoint, self.view, pickType='cells', tolerance=0.0)

        if not ret.pickedDataset:
            return None

        linkName = self.robotModel.model.getLinkNameForMesh(ret.pickedDataset)
        if not linkName:
            return None

        return ret.pickedPoint, linkName, ret.pickedNormal


    def onDoubleClick(self, displayPoint, modifiers=None):
        selection = self.getSelection(displayPoint)
        if selection is None:
            return False

        pickedPoint, linkName, normal = selection

        om.removeFromObjectModel(om.findObjectByName('link selection'))
        d = DebugData()
        d.addSphere(pickedPoint, radius=0.005)
        obj = vis.updatePolyData(d.getPolyData(), 'link selection', color=[1,0,0])
        obj.actor.SetPickable(False)

        self.selectedLink= linkName
        
        return True


app = mainwindowapp.construct()
view = app.view

shadowHandURDFFile = os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'models/shadow_hand/hand/shadow_hand_lite.urdf')

robotModel, jointController = roboturdf.loadRobotModel(urdfFile=shadowHandURDFFile, view=view, useConfigFile=False)
jointNames = robotModel.model.getJointNames()
jointController = jointcontrol.JointController([robotModel], jointNames=jointNames)
jointController.setZeroPose()

widget = ControlTargetControlWidget(view, robotModel)
widget.start()

treeviewer = treeviewer.TreeViewer(view)

app.app.start()