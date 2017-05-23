from director import objectmodel as om
from director import pointpicker
from director.tasks import robottasks

import iiwaplanning

class MyPlanner(object):

    def __init__(self, robotSystem, properties):
        self.properties = properties
        self.robotSystem = robotSystem
        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner
        self.objectPicker = pointpicker.ObjectPicker(robotSystem.view)
        self.affordance_name = None

    def reloadModule(self):
        import imp
        imp.reload(iiwaplanning)

    def fitObject(self):
        iiwaplanning.fitObjectOnSupport()
        iiwaplanning.addGraspFrames()

    def setAffordanceName(self, name):
        self.affordance_name = name

    def getAffordanceName(self):
        if self.affordance_name is not None:
            return self.affordance_name
        else:
            return self.properties.getPropertyEnumValue('Affordance name')

    def getGraspFrameSuffix(self):
        return self.graspFrameSuffix

    def fitSimulationObject(self):

        def onPick(objs):
            om.setActiveObject(objs[0])
            iiwaplanning.fitSelectedGeometryObject()

        self.objectPicker.stop()
        self.objectPicker.callbackFunc = onPick
        self.objectPicker.start()


    def spawnObject(self):
        iiwaplanning.spawnAffordance(self.getAffordanceName())

    def addGraspFrames(self):
        iiwaplanning.addGraspFrames(self.getAffordanceName())
        self.selectGraspFrameSuffix()

    def addBoxGraspFrames(self, graspOffset=None):
        print "adding box grasp frames"
        self.setAffordanceName('box')
        iiwaplanning.addBoxGraspFrames(graspOffset=graspOffset)
        self.selectGraspFrameSuffix()
        print "grasp frame suffix", self.getGraspFrameSuffix()

    def selectGraspFrameSuffix(self):
        costs = iiwaplanning.computeReachPlanCosts(self.getAffordanceName())
        print "costs", costs
        self.graspFrameSuffix = iiwaplanning.getBestGraspSuffix(costs)
        return costs[self.graspFrameSuffix].isFeasible

    def planGrasp(self):
        suffix = self.getGraspFrameSuffix()
        iiwaplanning.planReachGoal('grasp to world%s' % suffix)

    def planPreGrasp(self):
        suffix = self.getGraspFrameSuffix()
        print "planning pregrasp for ", 'pregrasp to world%s' % suffix
        iiwaplanning.planReachGoal('pregrasp to world%s' % suffix)

    def commitManipPlan(self):
        self.robotSystem.manipPlanner.commitManipPlan(self.robotSystem.ikPlanner.lastManipPlan)

    def waitForExecute(self):

        plan = self.robotSystem.ikPlanner.lastManipPlan
        lastPlanTime = self.robotSystem.planPlayback.getPlanElapsedTime(plan)
        yield robottasks.DelayTask(delayTime=lastPlanTime + 0.1).run()

    def openGripper(self):
        self.openGripperFunc()

    def closeGripper(self):
        self.closeGripperFunc()

    def planToNominal(self):
        iiwaplanning.planNominalPosture()
