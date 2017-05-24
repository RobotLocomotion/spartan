import math
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



def loadMomapModel(name, meshFile):
    momapObjectsRepo = os.path.expanduser('~/catkin_ws/src/perception_deps/objects')
    meshFileName = os.path.join(momapObjectsRepo, meshFile)

    return affordanceManager.newAffordanceFromDescription(
      dict(classname='MeshAffordanceItem', Name=name,
           pose=transformUtils.poseFromTransform(vtk.vtkTransform()),
           Filename=meshFileName))


def loadBlueFunnel():
    obj = loadMomapModel('blue funnel', 'meshes/blue_funnel.stl')
    setupTfFrameSync(obj, 'bfunnel')
    return obj


def setupTfFrameSync(obj, frameName):

    folder = om.findObjectByName('tf')
    if not folder:
        return

    frame = folder.findChild('bfunnel')
    obj.getChildFrame().copyFrame(frame.transform)
    obj.getChildFrame().getFrameSync().addFrame(frame)



def makeReachGoal(name='reach goal'):
    t = transformUtils.frameFromPositionAndRPY([0.7, 0.0, 0.3], [-90, 0, -90])
    return vis.updateFrame(t, 'reach goal').setProperty('Scale', 0.1)


def planNominalPosture():
    startPose = robotSystem.planningUtils.getPlanningStartPose()
    ikPlanner.computeDatabasePosturePlan(startPose, 'General', 'q_nom')


def getEndEffectorLinkName():

    config = drcargs.getDirectorConfig()['endEffectorConfig']
    linkName = config['endEffectorLinkNames'][0]
    assert linkName == ikPlanner.getHandLink()
    return linkName

def getGraspToHandLink():
    config = drcargs.getDirectorConfig()['endEffectorConfig']
    return transformUtils.frameFromPositionAndRPY(
                          config['graspOffsetFrame'][0],
                          np.degrees(config['graspOffsetFrame'][1]))

_callbackId = None

def planReachGoal(goalFrameName='reach goal', startPose=None, planTraj=True, interactive=False, seedFromStart=False):

    goalFrame = om.findObjectByName(goalFrameName).transform
    startPoseName = 'reach_start'
    endPoseName = 'reach_end'

    endEffectorLinkName = getEndEffectorLinkName()
    graspOffsetFrame = getGraspToHandLink()

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

    isPregrasp = goalFrameName.startswith('pregrasp to world')
    isGrasp = goalFrameName.startswith('grasp to world')
    if isGrasp:
        seedFromStart = True

    # adjust bounds of move on line constraint
    axisConstraintTubeRadius = 0.3 if isPregrasp else 0.001
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
    constraints.append(q)
    #constraints.append(g)
    #constraints.append(g2)
    constraints.append(axisConstraint)

    constraintSet = ikplanner.ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)

    constraintSet.ikParameters.usePointwise = True
    if seedFromStart:
        constraintSet.seedPoseName = startPoseName
        constraintSet.nominalPoseName = startPoseName
    elif isPregrasp:
        constraintSet.seedPoseName = 'q_nom'
        constraintSet.nominalPoseName = 'q_nom'

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

    pointInFrame = np.array(getGraspToHandLink().GetPosition())

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


def makeCylinder():
    ''' has properties Radius and Length '''
    desc = dict(classname='CylinderAffordanceItem',
                Name='cylinder',
                pose=transformUtils.poseFromTransform(vtk.vtkTransform()))
    return newAffordanceFromDescription(desc)


def makeBox():
    ''' has property Dimensions '''
    desc = dict(classname='BoxAffordanceItem',
                Name='box',
                pose=transformUtils.poseFromTransform(vtk.vtkTransform()))
    box = newAffordanceFromDescription(desc)
    box.getChildFrame().setProperty('Scale', 0.1)
    return box


def getPointCloud(name='openni point cloud'):
    obj = om.findObjectByName(name)
    return obj.polyData if obj else vtk.vtkPolyData()


def addHSVArrays(polyData, rgbArrayName='rgb_colors'):
    import colorsys
    rgb = vnp.getNumpyFromVtk(polyData, rgbArrayName)/255.0
    hsv = np.array([colorsys.rgb_to_hsv(*t) for t in rgb])
    vnp.addNumpyToVtk(polyData, hsv[:,0].copy(), 'hue')
    vnp.addNumpyToVtk(polyData, hsv[:,1].copy(), 'saturation')
    vnp.addNumpyToVtk(polyData, hsv[:,2].copy(), 'value')


def getMaxZCoordinate(polyData):
    return float(np.nanmax(vnp.getNumpyFromVtk(polyData, 'Points')[:,2]))


def fitSupport(pickPoint=[0.92858565, 0.00213802, 0.30315629]):

    om.removeFromObjectModel(om.findObjectByName('cylinder'))

    polyData = getPointCloud()

    t = vtk.vtkTransform()
    t.Translate(pickPoint)
    polyData = segmentation.cropToBox(polyData, t, [0.3,0.3,0.5])

    addHSVArrays(polyData)

    vis.updatePolyData(polyData, 'crop region', colorByName='rgb_colors', visible=False)

    zMax = getMaxZCoordinate(polyData)

    cyl = makeCylinder()
    cyl.setProperty('Radius', 0.03)
    cyl.setProperty('Length', zMax)

    origin = segmentation.computeCentroid(polyData)
    origin[2] = zMax/2.0

    t = transformUtils.frameFromPositionAndRPY(origin, [0,0,0])
    cyl.getChildFrame().copyFrame(t)


def cropToCylinder(polyData, p1, p2, radius):
    polyData = segmentation.cropToLineSegment(polyData, p1, p2)
    if polyData.GetNumberOfPoints():
        polyData = segmentation.labelDistanceToLine(polyData, p1, p2)
        polyData = segmentation.thresholdPoints(polyData, 'distance_to_line', [0.0, radius])
    return polyData


def getSupportSearchPoint(supportName='cylinder'):

    obj = om.findObjectByName(supportName)
    t = obj.getChildFrame().transform
    zCoord = obj.getProperty('Length')/2.0
    p = np.array(t.TransformPoint([0,0,zCoord]))
    return p


def extractSearchRegionAboveSupport():
    polyData = getPointCloud()
    p1 = getSupportSearchPoint()
    p2 = p1 + np.array([0,0,1.0])*0.3
    polyData = cropToCylinder(polyData, p1, p2, radius=0.1)
    return polyData


def spawnAffordance(affordanceName):
    dispatch = {
        'box' : spawnBox,
        'blue funnel' : spawnBlueFunnel
        }
    dispatch[affordanceName]()


def spawnBox():

    t = transformUtils.frameFromPositionAndRPY([0.5,0.0,0.5], [0,0,0])

    om.removeFromObjectModel(om.findObjectByName('box'))
    obj = makeBox()
    obj.setProperty('Dimensions', [0.06, 0.04, 0.12])
    #obj.setProperty('Surface Mode', 'Wireframe')
    obj.setProperty('Color', [1,0,0])
    obj.getChildFrame().copyFrame(t)


def spawnBlueFunnel():

    t = transformUtils.frameFromPositionAndRPY([0.5,-0.25,0.2], [-90,0,0])

    om.removeFromObjectModel(om.findObjectByName('blue funnel'))
    obj = loadBlueFunnel()
    obj.setProperty('Color', [0.0, 0.5, 1.0])
    obj.getChildFrame().copyFrame(t)


def fitSelectedGeometryObject():
    obj = om.getActiveObject()
    t = obj.actor.GetUserTransform()
    polyData = filterUtils.transformPolyData(obj.polyData, t)
    obj = fitObjectFromPolyData(polyData)


def fitObjectOnSupport():

    polyData = extractSearchRegionAboveSupport()
    fitObjectFromPolyData(polyData)


def fitObjectFromPolyData(polyData):

    origin, edges, wireframe = segmentation.getOrientedBoundingBox(polyData)

    edgeLengths = [float(np.linalg.norm(edge)) for edge in edges]
    axes = [edge / np.linalg.norm(edge) for edge in edges]

    boxCenter = segmentation.computeCentroid(wireframe)

    t = transformUtils.getTransformFromAxes(axes[0], axes[1], axes[2])
    zAxisIndex, zAxis, zAxisSign = transformUtils.findTransformAxis(t, [0,0,1.0])
    xAxisIndex, xAxis, xAxisSign = transformUtils.findTransformAxis(t, [1.0,0,0])

    assert zAxisIndex != xAxisIndex

    #zAxis = zAxis*zAxisSign
    zAxis = [0,0,1.0]
    xAxis = xAxis*xAxisSign
    yAxis = np.cross(zAxis, xAxis)
    xAxis = np.cross(yAxis, zAxis)

    zLength = edgeLengths[zAxisIndex]
    xLength = edgeLengths[xAxisIndex]
    ids = [0,1,2]
    ids.remove(zAxisIndex)
    ids.remove(xAxisIndex)
    yLength = edgeLengths[ids[0]]


    t = transformUtils.getTransformFromAxes(xAxis, yAxis, zAxis)
    t.PostMultiply()
    t.Translate(boxCenter)

    om.removeFromObjectModel(om.findObjectByName('box'))
    obj = makeBox()
    obj.setProperty('Dimensions', [xLength, yLength, zLength])
    obj.getChildFrame().copyFrame(t)
    #obj.getChildFrame().setProperty('Visible', True)
    obj.setProperty('Surface Mode', 'Wireframe')
    obj.setProperty('Color', [1,0,0])
    return obj


def setBoxGraspTarget(position, rpy, dimensions):
    rot_quat = transformUtils.rollPitchYawToQuaternion(
        [math.radians(x) for x in rpy])
    t = transformUtils.transformFromPose(position, rot_quat)

    om.removeFromObjectModel(om.findObjectByName('box'))
    obj = makeBox()
    obj.setProperty('Dimensions', dimensions)
    obj.getChildFrame().copyFrame(t)
    obj.setProperty('Surface Mode', 'Wireframe')
    obj.setProperty('Color', [1,0,0])


def addGraspFrames(affordanceName='box'):
    dispatch = {
        'box' : addBoxGraspFrames,
        'blue funnel' : addFunnelGraspFrames
        }
    dispatch[affordanceName]()


def makePostGraspFrame(obj, graspFrameName):
    graspFrame = om.findObjectByName(graspFrameName).transform
    goalTransform = vtk.vtkTransform()
    goalTransform.Translate(0, 0, 0.10)
    # Copy the resulting matrix.  graspFrame can move around if the
    # robot's frame moves when the gripper occludes the mocap.
    goalTransform.SetMatrix(transformUtils.concatenateTransforms([
        graspFrame, goalTransform]).GetMatrix())

    goalFrameName = "postgrasp to world"
    om.removeFromObjectModel(om.findObjectByName(goalFrameName))

    goalFrame = vis.showFrame(goalTransform, goalFrameName, scale=0.1, parent=obj, visible=False)
    obj.getChildFrame().getFrameSync().addFrame(goalFrame, ignoreIncoming=True)


def makeGraspFrames(obj, graspOffset, pregraspOffset=(-0.08, 0, 0), suffix=''):

    pos, rpy = graspOffset
    objectToWorld = obj.getChildFrame().transform
    graspToObject = transformUtils.frameFromPositionAndRPY(pos, rpy)
    preGraspToGrasp = transformUtils.frameFromPositionAndRPY(pregraspOffset, [0,0,0])
    graspToWorld = transformUtils.concatenateTransforms([graspToObject, objectToWorld])
    preGraspToWorld = transformUtils.concatenateTransforms([preGraspToGrasp, graspToWorld])

    graspFrameName = 'grasp to world%s' % suffix
    pregraspFrameName = 'pregrasp to world%s' % suffix

    om.removeFromObjectModel(om.findObjectByName(graspFrameName))
    om.removeFromObjectModel(om.findObjectByName(pregraspFrameName))

    graspFrame = vis.showFrame(graspToWorld, graspFrameName, scale=0.1, parent=obj, visible=False)
    preGraspFrame = vis.showFrame(preGraspToWorld, pregraspFrameName, scale=0.1, parent=obj, visible=False)

    obj.getChildFrame().getFrameSync().addFrame(graspFrame, ignoreIncoming=True)
    graspFrame.getFrameSync().addFrame(preGraspFrame, ignoreIncoming=True)


def addFunnelGraspFrames():

    obj = om.findObjectByName('blue funnel')

    # y axis points down into the funnel
    # x/z axes points along funnel width/length dimensions
    originToPinch = -0.05
    graspOffsets = [
        ([-0.04, originToPinch, 0.0], [0,0,90]),
        ([-0.04, originToPinch, 0.0], [180,0,90]),

        ([0.04, originToPinch, 0.0], [180,0,90]),
        ([0.04, originToPinch, 0.0], [0,0,90]),

        ([0.0, originToPinch, -0.07], [-90,0,90]),
        ([0.0, originToPinch, -0.07], [90,0,90]),

        ([0.0, originToPinch, 0.07], [90,0,90]),
        ([0.0, originToPinch, 0.07], [-90,0,90]),
        ]

    for i, graspOffset in enumerate(graspOffsets):
        makeGraspFrames(obj, graspOffset, suffix=' %d' % i)


def addBoxGraspFrames(graspOffset=None):
    obj = om.findObjectByName('box')
    if graspOffset is None:
        dims = obj.getProperty('Dimensions')
        graspOffset = ([0.0, 0.0, dims[2]/2.0 - 0.025], [0,0,0])
    makeGraspFrames(obj, graspOffset, pregraspOffset=(0.0, 0.0, 0.08))


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
        #if not robotSystem.planPlayback.isPlanInfoFeasible(info):
        if not (0 <= info < 10):
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


def moveFunnelRandomly():

    xyz = [
        np.random.uniform(0.25, 0.75),
        np.random.uniform(-0.3, 0.3),
        np.random.uniform(0.0, 0.5)]

    rpy = [
        np.random.uniform(-120, -60),
        np.random.uniform(-30, 30),
        np.random.uniform(-30, 30)]


    t = transformUtils.frameFromPositionAndRPY(xyz, rpy)
    om.findObjectByName('blue funnel').getChildFrame().copyFrame(t)


def randomTest():
    moveFunnelRandomly()
    makeBestPlan('blue funnel')
