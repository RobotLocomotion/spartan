from director import objectmodel as om
from director import ioUtils
from director import visualization as vis
from director import transformUtils
from director import filterUtils
from director import segmentation
from director import pointpicker
from director import lcmframe
from director import lcmUtils
from director import applogic
from director.debugVis import DebugData
from director import viewbehaviors
from director import vtkAll as vtk
from director import vtkNumpy as vnp
from director.shallowCopy import shallowCopy
import numpy as np
from matplotlib.mlab import PCA
import time

import PythonQt
from PythonQt import QtGui, QtCore

import irb140planning
import robotiqhand

def sendGripperCommand(targetPositionMM, force):
    #msg = lcmdrake.lcmt_schunk_wsg_command()
    #msg.utime = int(time.time()*1e6)
    #msg.force = force
    #msg.target_position_mm = targetPositionMM
    #lcmUtils.publish('SCHUNK_WSG_COMMAND', msg)
    msg = robotiqhand.command_t()
    msg.utime = int(time.time()*1e6)
    msg.activate=1
    msg.emergency_release = 0
    msg.do_move = 1
    msg.mode = 1
    msg.position = targetPositionMM
    msg.force = force
    msg.velocity = 100
    msg.ifc = 0
    msg.isc = 0
    lcmUtils.publish("ROBOTIQ_LEFT_COMMAND", msg)

# Visually dissimilar colors
# https://graphicdesign.stackexchange.com/questions/3682/where-can-i-find-a-large-palette-set-of-contrasting-colors-for-coloring-many-d
color_set = [
    [255,0,0], 
    [228,228,0], 
    [0,255,0], 
    [0,255,255], 
    [176,176,255], 
    [255,0,255], 
    [228,228,228], 
    [176,0,0], 
    [186,186,0], 
    [0,176,0], 
    [0,176,176], 
    [132,132,255], 
    [176,0,176], 
    [186,186,186], 
    [135,0,0], 
    [135,135,0], 
    [0,135,0], 
    [0,135,135], 
    [73,73,255], 
    [135,0,135], 
    [135,135,135], 
    [85,0,0], 
    [84,84,0], 
    [0,85,0], 
    [0,85,85], 
    [0,0,255], 
    [85,0,85], 
    [84,84,84]
]
def get_ith_color(i):
    return np.array(color_set[i % len(color_set)])/255.

def doTabletopSegmentation(tableTransform, save=False):
    om.removeFromObjectModel(om.findObjectByName('table'))
        
    obj = vis.updateFrame(tableTransform, "tableTransform", parent='table')

    pc = om.findObjectByName('openni point cloud')
    polyData = pc.polyData

    viewPlaneNormal = tableTransform.TransformNormal(0, 0, 1)
    viewPlaneOrigin = transformUtils.poseFromTransform(tableTransform)[0]
    print viewPlaneOrigin, viewPlaneNormal

    polyData, origin, normal = segmentation.applyPlaneFit(
        polyData, 
        expectedNormal=viewPlaneNormal,
        searchOrigin=viewPlaneOrigin,
        searchRadius=0.4,
        angleEpsilon=0.1,
        returnOrigin=True)

    tablePoints = segmentation.thresholdPoints(
        polyData, 
        'dist_to_plane',
        [-0.01, 0.01])

    vis.updatePolyData(
        tablePoints,
        'table plane points',
        parent="table",
        color=[0.5,0.0,0.0],
        visible=False)

    # Points above the table, and close to the tag of the tag
    searchPoints = segmentation.thresholdPoints(polyData, 'dist_to_plane', [0.01, 0.4])
    searchPoints = segmentation.cropToSphere(searchPoints, viewPlaneOrigin, 0.4)
    
    vis.updatePolyData(
        searchPoints,
        'object points',
        parent="table",
        color=[0,0.0,0.5],
        visible=False)


    searchPoints = segmentation.labelDistanceToPoint(searchPoints, viewPlaneOrigin)
    searchPointsClusters = segmentation.extractClusters(searchPoints, clusterTolerance=0.01)
    searchPointsClusters.sort(key=lambda x: vnp.getNumpyFromVtk(x, 'distance_to_point').min())

    for i, cluster in enumerate(searchPointsClusters):
        pd = vis.updatePolyData(cluster, 'table points %d' % i, parent="table", color=get_ith_color(i), visible=True)
        pd.setProperty('Point Size', 5)

    if save:
        import datetime
        prefix = datetime.datetime.now().isoformat()
        print "Saving %s...vtp" % prefix
        ioUtils.writePolyData(pc.polyData, prefix + "_original.vtp")
        for i, cluster in enumerate(searchPointsClusters):
            ioUtils.writePolyData(cluster, prefix + "_table_points_%d.vtp" % i)

def doReachToPointCluster(point_cluster_name = "table points 0"):
    cluster = om.findObjectByName(point_cluster_name)

    # Align a frame to this segmentation, with the gripper's +z along the
    # long axis of the object, and +x straight down, centered on
    # centroid of the segmentation

    results = PCA(vnp.getNumpyFromVtk(cluster.polyData))
    z_axis = results.Wt[0]
    z_axis[2] = 0.
    z_axis /= np.linalg.norm(z_axis)
    x_axis = np.array([0., 0., -1.])
    y_axis = np.cross(z_axis, x_axis)

    pca_tf = transformUtils.getTransformFromAxes(x_axis, y_axis, z_axis)
    pca_tf.PostMultiply()
    pca_tf.Translate(results.mu)

    pca_tf_reach = transformUtils.copyFrame(pca_tf)
    pca_tf_reach.Translate([0., 0., 0.2])
    reach_goal = irb140planning.makeReachGoalFrame(name='reach goal', tf=pca_tf_reach)
    grasp_goal = irb140planning.makeReachGoalFrame(name='grasp goal', tf=pca_tf)