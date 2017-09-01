from director import transformUtils
from director import lcmUtils
from director import objectmodel as om
from director import visualization as vis
from director import applogic as app
from director import ikplanner
from director import vtkNumpy as vnp
import director.vtkAll as vtk
from director.debugVis import DebugData
from director.ikparameters import IkParameters
from director import pointpicker
from director import segmentation

from director.tasks import robottasks as rt
from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import os
import sys
import time
import numpy as np
from PythonQt import QtCore, QtGui

import irb140planning
import tabletop_manipulation_tool


class MyPlanner(object):

    def __init__(self, robotSystem, properties):
        self.properties = properties
        self.robotSystem = robotSystem
        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner
        self.objectPicker = pointpicker.ObjectPicker(robotSystem.view)
        
        irb140planning.makeReachGoalFrame(name="drop goal")
        self.dropGoalValid = False

    def reloadModule(self):
        import imp
        imp.reload(irb140planning)
        imp.reload(tabletop_manipulation_tool)
    def resetDropFrame(self):
        tableTagToWorld = om.findObjectByName("table tag to world").transform
        self.dropFrame = tableTagToWorld
        self.dropGoalValid = True

    def doTabletopSegmentation(self):
        tableTagToWorld = om.findObjectByName("table tag to world").transform
        tableTagFlipped = transformUtils.concatenateTransforms([
        transformUtils.frameFromPositionAndRPY([0, 0, 0], [180, 0, 0]),
        tableTagToWorld])
        tabletop_manipulation_tool.doTabletopSegmentation(tableTagFlipped)
        tabletop_manipulation_tool.doReachToPointCluster()

    def doTabletopSegmentationAndSave(self):
        tableTagToWorld = om.findObjectByName("table tag to world").transform
        tableTagFlipped = transformUtils.concatenateTransforms([
        transformUtils.frameFromPositionAndRPY([0, 0, 0], [180, 0, 0]),
        tableTagToWorld])
        tabletop_manipulation_tool.doTabletopSegmentation(tableTagFlipped, save=True)
        tabletop_manipulation_tool.doReachToPointCluster()

    def planReach(self):
        irb140planning.planToReachGoal('reach goal')

    def planGrasp(self):
        irb140planning.planToReachGoal('grasp goal')

    def planDrop(self):
        if self.dropGoalValid:
            irb140planning.planToReachGoal('drop goal')
        else:
            irb140planning.planToReachGoal('reach goal')

    def planNominal(self):
        irb140planning.planNominalPosture()

    def commitManipPlan(self):
        self.robotSystem.manipPlanner.commitManipPlan(self.robotSystem.ikPlanner.lastManipPlan)

    def waitForExecute(self):
        plan = self.robotSystem.ikPlanner.lastManipPlan
        lastPlanTime = self.robotSystem.planPlayback.getPlanElapsedTime(plan)
        yield rt.DelayTask(delayTime=lastPlanTime*4).run()

    def openGripper(self):
        tabletop_manipulation_tool.sendGripperCommand(0, 125)

    def closeGripper(self):
        tabletop_manipulation_tool.sendGripperCommand(255, 125)


class LoneliestTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Lonely Task Panel')

        rt.robotSystem = robotSystem
        irb140planning.init(robotSystem)

        self.planner = MyPlanner(robotSystem, self.params)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):
        self.addManualButton('Run Tabletop Segmentation', self.planner.doTabletopSegmentation)
        self.addManualButton('Save Tabletop Segmentation', self.planner.doTabletopSegmentationAndSave)
        self.addManualButton('Reach segment 0', tabletop_manipulation_tool.doReachToPointCluster)
        self.addManualSpacer()
        self.addManualButton('Plan Reach', self.planner.planReach)
        self.addManualButton('Plan Grasp', self.planner.planGrasp)
        self.addManualButton('Plan Above Table', self.planner.planDrop)
        self.addManualButton('Plan Nominal', self.planner.planNominal)
        self.addManualSpacer()
        self.addManualButton('Gripper Open', self.planner.openGripper)
        self.addManualButton('Gripper Close', self.planner.closeGripper)
        self.addManualSpacer()
        self.addManualButton('Reload module', self.planner.reloadModule)
        self.addManualButton('Reset drop frame', self.planner.resetDropFrame)

    def addDefaultProperties(self):
        pass

    def onPropertyChanged(self, propertySet, propertyName):
        pass

    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(name, func, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)

        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addManipTask(name, planFunc, userPrompt=False):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc('plan', planFunc)
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc('execute manip plan', self.planner.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder

        self.taskTree.removeAllTasks()
        ##################

        addFolder('reset')
        addFunc('plan to nominal', self.planner.planNominal)
        addFunc('execute', self.planner.commitManipPlan)
        addFunc('wait for execute', self.planner.waitForExecute)
        addFunc('open gripper', self.planner.openGripper)
        addTask(rt.PauseTask(name='pause'))

        addFolder('grasp loop')

        addFunc('do segmentation', self.planner.doTabletopSegmentation)

        addFunc('plan pregrasp', self.planner.planReach)
        addFunc('execute', self.planner.commitManipPlan)
        addFunc('wait for execute', self.planner.waitForExecute)
        addFunc('plan grasp', self.planner.planGrasp)
        addFunc('execute', self.planner.commitManipPlan)
        addFunc('wait for execute', self.planner.waitForExecute)
        addFunc('close gripper', self.planner.closeGripper)
        addTask(rt.DelayTask(name='wait', delayTime=1.0))

        addFunc('plan to drop', self.planner.planDrop)
        addFunc('execute', self.planner.commitManipPlan)
        addFunc('wait for execute', self.planner.waitForExecute)
        addFunc('open gripper', self.planner.openGripper)

        addFunc('plan to nominal', self.planner.planNominal)
        addFunc('execute', self.planner.commitManipPlan)
        addFunc('wait for execute', self.planner.waitForExecute)
        addFunc('open gripper', self.planner.openGripper)
