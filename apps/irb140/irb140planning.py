import os

from director import robotstate
from director import drcargs
from director.fieldcontainer import FieldContainer
from director import filterUtils
from director import transformUtils
from director import ikplanner
from director import ikconstraints
from director import lcmUtils
from director import segmentation
from director import visualization as vis
from director import objectmodel as om
from director.debugVis import DebugData
from director import vtkAll as vtk
from director import vtkNumpy as vnp
import numpy as np

def makeReachGoalFrame(name='reach goal', tf=None):
    t = tf or transformUtils.frameFromPositionAndRPY([0.7, 0.0, 0.3], [-90, 0, -90])
    return vis.updateFrame(t, name).setProperty('Scale', 0.1)

def planNominalPosture():
    startPose = robotSystem.planningUtils.getPlanningStartPose()
    ikPlanner.computeDatabasePosturePlan(startPose, 'General', 'Nominal, All Zeros')

def getEndEffectorLinkName():
    config = drcargs.getDirectorConfig()['endEffectorConfig']
    linkName = config['endEffectorLinkNames'][0]
    assert linkName == ikPlanner.getHandLink()
    return linkName

def getGraspToHandLinkTransform():
    config = drcargs.getDirectorConfig()['endEffectorConfig']
    return transformUtils.frameFromPositionAndRPY(
                          config['graspOffsetFrame'][0],
                          np.degrees(config['graspOffsetFrame'][1]))

_callbackId = None

def planToReachGoal(goalFrameName='reach goal', startPose=None, planTraj=True, interactive=False):
    goalFrame = om.findObjectByName(goalFrameName).transform
    startPoseName = 'reach_start'
    endPoseName = 'reach_end'

    endEffectorLinkName = getEndEffectorLinkName()
    graspOffsetFrame = getGraspToHandLinkTransform()

    if startPose is None:
        startPose = robotSystem.planningUtils.getPlanningStartPose()
    ikPlanner.addPose(startPose, startPoseName)

    constraints = []
    constraints.append(ikPlanner.createPostureConstraint(startPoseName, robotstate.matchJoints('base_')))
    p, q = ikPlanner.createPositionOrientationConstraint(endEffectorLinkName, goalFrame, graspOffsetFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0)
    p.tspan = [1.0, 1.0]
    q.tspan = [1.0, 1.0]

    _, _, axisConstraint = ikPlanner.createMoveOnLineConstraints(startPose, goalFrame, graspOffsetFrame)

    axisConstraint.tspan = np.linspace(0,1,10)

    isPregrasp = goalFrameName.startswith('pre')
    #isPregrasp = False

    # adjust bounds of move on line constraint
    axisConstraintTubeRadius = 0.1 if isPregrasp else 0.0
    axisConstraint.lowerBound[0] = -axisConstraintTubeRadius
    axisConstraint.lowerBound[0] = -axisConstraintTubeRadius
    axisConstraint.upperBound[1] = axisConstraintTubeRadius
    axisConstraint.upperBound[1] = axisConstraintTubeRadius

    # allow sliding in Z axis of pinch frame
    # this may be overruled by the line constraint
    if isPregrasp:
        p.lowerBound[2] = -0.02
        p.upperBound[2] = 0.02

    # align the gripper pinch axis
    # with the Y axis of the goal frame
    g = ikconstraints.WorldGazeDirConstraint()
    g.linkName = endEffectorLinkName
    g.targetFrame = goalFrame
    g.targetAxis = [0,1,0]
    g.bodyAxis = list(graspOffsetFrame.TransformVector([0,1,0]))
    g.coneThreshold = np.radians(5.0) if isPregrasp else np.radians(0.0)
    g.tspan = [1.0, 1.0]

    # point the fingers along the X axis
    # of the goal frame
    pinchPivotBound = np.radians(20) if isPregrasp else np.radians(0)
    g2 = ikconstraints.WorldGazeDirConstraint()
    g2.linkName = endEffectorLinkName
    g2.targetFrame = goalFrame
    g2.targetAxis = [1,0,0]
    g2.bodyAxis = list(graspOffsetFrame.TransformVector([1,0,0]))
    g2.coneThreshold = pinchPivotBound
    g2.tspan = [1.0, 1.0]


    constraints.append(p)
    constraints.append(g)
    constraints.append(g2)
    constraints.append(axisConstraint)

    constraintSet = ikplanner.ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)

    constraintSet.ikParameters.usePointwise = True

    global _callbackId
    #if _callbackId:
    #    om.findObjectByName(goalFrameName).disconnectFrameModified(_callbackId)

    if interactive:
        def update(frame):
            endPose, info = constraintSet.runIk()
            robotSystem.teleopPanel.showPose(endPose)

        _callbackId = om.findObjectByName(goalFrameName).connectFrameModified(update)
        update(None)

    else:
        endPose, info = constraintSet.runIk()

        if planTraj:
            robotSystem.teleopPanel.hideTeleopModel()
            return constraintSet.runIkTraj()
        else:
            return endPose, info


def getLinkFrameSamplesFromPlan(plan, linkName, numberOfSamples=30):

    poseTimes, poses = robotSystem.planPlayback.getPlanPoses(plan)
    poseInterpolator = robotSystem.planPlayback.getPoseInterpolator(poseTimes, poses)

    sampleTimes = np.linspace(poseTimes[0], poseTimes[-1], numberOfSamples)

    frames = []
    for sampleTime in sampleTimes:
        pose = poseInterpolator(sampleTime)
        linkFrame = ikPlanner.getLinkFrameAtPose(linkName, pose)
        frames.append(linkFrame)

    return frames


def drawEndEffectorTrajFromPlan(plan=None):

    if plan is None:
        plan = robotSystem.ikPlanner.lastManipPlan
    linkName = getEndEffectorLinkName()

    frames = getLinkFrameSamplesFromPlan(plan, linkName)

    pointInFrame = np.array(getGraspToHandLinkTransform().GetPosition())

    pts = []
    for f in frames:
        p = np.array(f.TransformPoint(pointInFrame))
        pts.append(p)

    d = DebugData()
    for p1, p2 in zip(pts, pts[1:]):
        d.addLine(p1, p2)

    vis.updatePolyData(d.getPolyData(), 'end effector traj', parent='debug')


def showDebugPoint(p, name='debug point', update=False, visible=True):
    d = DebugData()
    d.addSphere(p, radius=0.01, color=[1,0,0])
    if update:
        vis.updatePolyData(d.getPolyData(), name)
    else:
        vis.showPolyData(d.getPolyData(), name, colorByName='RGB255', visible=visible)


def makeGraspFrames(obj, graspOffset, pregraspOffset=-0.08, suffix=''):

    pos, rpy = graspOffset
    objectToWorld = obj.getChildFrame().transform
    graspToObject = transformUtils.frameFromPositionAndRPY(pos, rpy)
    preGraspToGrasp = transformUtils.frameFromPositionAndRPY([pregraspOffset, 0.0, 0.0], [0,0,0])
    graspToWorld = transformUtils.concatenateTransforms([graspToObject, objectToWorld])
    preGraspToWorld = transformUtils.concatenateTransforms([preGraspToGrasp, graspToWorld])

    graspFrameName = 'grasp to world%s' % suffix
    pregraspFrameName = 'pregrasp to world%s' % suffix

    om.removeFromObjectModel(obj.findChild(graspFrameName))
    om.removeFromObjectModel(obj.findChild(pregraspFrameName))

    graspFrame = vis.showFrame(graspToWorld, graspFrameName, scale=0.1, parent=obj, visible=False)
    preGraspFrame = vis.showFrame(preGraspToWorld, pregraspFrameName, scale=0.1, parent=obj, visible=False)

    obj.getChildFrame().getFrameSync().addFrame(graspFrame, ignoreIncoming=True)
    graspFrame.getFrameSync().addFrame(preGraspFrame, ignoreIncoming=True)


def init(robotSystem_):
    global robotSystem, affordanceManager, ikPlanner, newAffordanceFromDescription

    robotSystem = robotSystem_
    affordanceManager = robotSystem.affordanceManager
    ikPlanner = robotSystem.ikPlanner
    newAffordanceFromDescription = robotSystem.affordanceManager.newAffordanceFromDescription



#############################


def computePoseCost(pose):
    return np.linalg.norm(robotSystem.ikPlanner.jointController.getPose('q_nom') - pose)


def computeReachPlanCost(suffix):

    goalNames = ['pregrasp to world%s' % suffix,
                 'grasp to world%s' % suffix]

    endPose = None
    isFeasible = True
    cost = 0.0
    for goalName in goalNames:
        endPose, info = planReachGoal(goalName, startPose=endPose, planTraj=False)
        cost += computePoseCost(endPose)
        if not robotSystem.planPlayback.isPlanInfoFeasible(info):
            isFeasible = False

    return FieldContainer(cost=cost, isFeasible=isFeasible, endPose=endPose)


def getGraspFrameSuffixes(affordanceName):

    obj = om.findObjectByName(affordanceName)

    suffixes = []
    prefix = 'grasp to world'

    for child in obj.children():
        name = child.getProperty('Name')
        if name.startswith(prefix):
            suffixes.append(name[len(prefix):])

    return suffixes


def computeReachPlanCosts(affordanceName):

    suffixes = getGraspFrameSuffixes(affordanceName)
    costs = {}
    for suffix in suffixes:
        costs[suffix] = computeReachPlanCost(suffix)
    return costs


def showCostResult(result):
    print 'is feasible:', result.isFeasible
    print 'cost:', result.cost
    robotSystem.teleopPanel.showPose(result.endPose)


def makeCombinedReachPlan(suffix=''):

    goalNames = ['pregrasp to world%s' % suffix,
                 'grasp to world%s' % suffix]

    plans = []

    for goalName in goalNames:

        if plans:
            _, poses = robotSystem.planPlayback.getPlanPoses(plans[-1])
            startPose = poses[-1]
        else:
            startPose = None

        plan = planReachGoal(goalName, startPose=startPose)
        plans.append(plan)

    plan = robotSystem.planPlayback.mergePlanMessages(plans)
    lcmUtils.publish('CANDIDATE_MANIP_PLAN', plan)
    drawEndEffectorTrajFromPlan(plan)

def getBestGraspSuffix(costs):
    sortedNames = sorted(costs.keys(), key=lambda x:costs[x].cost)
    for suffix in sortedNames:
        if costs[suffix].isFeasible:
            break
    else:
        suffix = sortedNames[0]
    return suffix

def makeBestPlan(affordanceName):

    costs = computeReachPlanCosts(affordanceName)
    suffix = getBestGraspSuffix(costs)

    if not costs[suffix].isFeasible:
        robotSystem.playbackRobotModel.setProperty('Color', [1, 0.25, 0.25])
    else:
        robotSystem.playbackRobotModel.setProperty('Color', [1, 0.75, 0])

    makeCombinedReachPlan(suffix)
