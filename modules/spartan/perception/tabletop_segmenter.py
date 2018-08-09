#!/usr/bin/env python
from __future__ import print_function

import argparse
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

import geometry_msgs.msg
import ros_numpy
import rospy
import sensor_msgs.msg

import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils

from director import filterUtils
from director import ioUtils
from director import transformUtils
from director.shallowCopy import shallowCopy
from director import vtkNumpy
from director import vtkAll as vtk

import meshcat
import meshcat.geometry as meshcat_g
import meshcat.transformations as meshcat_tf


def pointcloud2ToVtk(pc2):
    pc_np = ros_numpy.numpify(pc2)
    n_points = pc_np.shape[0]*pc_np.shape[1]
    points = np.zeros((n_points, 3))
    points[:, 0] = pc_np['x'].flatten()
    points[:, 1] = pc_np['y'].flatten()
    points[:, 2] = pc_np['z'].flatten()
    colors = np.frombuffer(
        pc_np['rgb'].flatten(), dtype=np.uint8, count=3*n_points)
    colors = colors.reshape(n_points, 3)
    good_entries = np.logical_not(np.isnan(points, ).any(axis=1))
    points = points[good_entries, :]
    colors = colors[good_entries, :]
    # TODO: what's the right way to cram color into a polydata?
    pc_vtk = vtkNumpy.numpyToPolyData(
        points, pointData=None, createVertexCells=True)
    return pc_vtk, colors


def applyEuclideanClustering(dataObj, clusterTolerance=0.05,
                             minClusterSize=100, maxClusterSize=1e6):
    # COPIED FROM DIRECTOR/SEGMENTATIONROUTINES
    # (which I can't import because importing PythonQt is broken)
    f = vtk.vtkPCLEuclideanClusterExtraction()
    f.SetInputData(dataObj)
    f.SetClusterTolerance(clusterTolerance)
    f.SetMinClusterSize(int(minClusterSize))
    f.SetMaxClusterSize(int(maxClusterSize))
    f.Update()
    return shallowCopy(f.GetOutput())


def extractClusters(polyData, clusterInXY=False, **kwargs):
    ''' Segment a single point cloud into smaller clusters
        using Euclidean Clustering
     '''

    if not polyData.GetNumberOfPoints():
        return []

    if clusterInXY is True:
        ''' If Points are seperated in X&Y, then cluster outside this '''
        polyDataXY = vtk.vtkPolyData()
        polyDataXY.DeepCopy(polyData)
        points = vtkNumpy.getNumpyFromVtk(polyDataXY, 'Points')
        points[:, 2] = 0.0
        polyDataXY = applyEuclideanClustering(polyDataXY, **kwargs)
        clusterLabels = vtkNumpy.getNumpyFromVtk(polyDataXY, 'cluster_labels')
        vtkNumpy.addNumpyToVtk(polyData, clusterLabels, 'cluster_labels')

    else:
        polyData = applyEuclideanClustering(polyData, **kwargs)
        clusterLabels = vtkNumpy.getNumpyFromVtk(polyData, 'cluster_labels')

    clusters = []
    for i in xrange(1, clusterLabels.max() + 1):
        cluster = filterUtils.thresholdPoints(polyData, 'cluster_labels',
                                              [i, i])
        clusters.append(cluster)
    return clusters


def applyVoxelGrid(polyData, leafSize=0.01):

    v = vtk.vtkPCLVoxelGrid()
    v.SetLeafSize(leafSize, leafSize, leafSize)
    v.SetInputData(polyData)
    v.Update()
    return shallowCopy(v.GetOutput())


def getMajorPlanes(polyData, useVoxelGrid=False,
                   voxelGridSize=0.01,
                   distanceToPlaneThreshold=0.02):

    if useVoxelGrid:
        polyData = applyVoxelGrid(polyData, leafSize=voxelGridSize)

    polyDataList = []

    minClusterSize = 100

    planeInfo = []
    while len(polyDataList) < 5:
        f = vtk.vtkPCLSACSegmentationPlane()
        f.SetInputData(polyData)
        f.SetDistanceThreshold(distanceToPlaneThreshold)
        f.Update()
        polyData = shallowCopy(f.GetOutput())

        outliers = filterUtils.thresholdPoints(polyData, 'ransac_labels', [0, 0])
        inliers = filterUtils.thresholdPoints(polyData, 'ransac_labels', [1, 1])
        largestCluster = extractLargestCluster(inliers)

        if largestCluster.GetNumberOfPoints() > minClusterSize:
            polyDataList.append(largestCluster)
            polyData = outliers
            planeInfo.append((np.array(f.GetPlaneOrigin()),
                              np.array(f.GetPlaneNormal())))
        else:
            break

    return polyDataList, planeInfo


def extractLargestCluster(polyData, **kwargs):
    '''
    Calls applyEuclideanClustering and then extracts the first (largest) cluster.
    The given keyword arguments are passed into the applyEuclideanClustering function.
    '''
    polyData = applyEuclideanClustering(polyData, **kwargs)
    return filterUtils.thresholdPoints(polyData, 'cluster_labels', [1, 1])


def draw_polydata_in_meshcat(vis, polyData, name, color=None, size=0.001):
    points = vtkNumpy.getNumpyFromVtk(polyData).T
    if color is not None:
        colors = np.tile(color[0:3], [points.shape[1], 1]).T
    else:
        colors = np.zeros(points.shape) + 1.
    vis["perception"]["tabletopsegmenter"][name].set_object(
        meshcat_g.PointCloud(position=points, color=colors, size=size))


def get_tf_from_point_normal(point, normal):
    tf = np.eye(4)
    # From https://math.stackexchange.com/questions/1956699/getting-a-transformation-matrix-from-a-normal-vector
    nx, ny, nz = normal
    if (nx**2 + ny**2 != 0.):
        nxny = np.sqrt(nx**2 + ny**2)
        tf[0, 0] = ny / nxny
        tf[0, 1] = -nx / nxny
        tf[1, 0] = nx*nz / nxny
        tf[1, 1] = ny*nz / nxny
        tf[1, 2] = -nxny
        tf[2, :3] = normal
    tf[:3, :3] = tf[:3, :3].T
    tf[0:3, 3] = np.array(point)
    return tf


def draw_plane_in_meshcat(vis, point, normal, size, name):
    size = np.array(size)
    vis["perception"]["tabletopsegmenter"][name].set_object(
        meshcat_g.Box(size))

    box_tf = get_tf_from_point_normal(point, normal)
    vis["perception"]["tabletopsegmenter"][name].set_transform(box_tf)


class TabletopObjectSegmenter:
    def __init__(self,
                 zmq_url="tcp://127.0.0.1:6000",
                 visualize=False):

        print("Opening meshcat vis... will hang if no server"
              "\tis running.")
        self.vis = None
        if visualize:
            self.vis = meshcat.Visualizer(zmq_url=zmq_url)
            self.vis["perception"].delete()
            self.vis["perception/tabletopsegmenter"].delete()

    def segment_pointcloud(self, polyData, visualize=False):
        polyDataSimplified = applyVoxelGrid(polyData, leafSize=0.005)
        draw_polydata_in_meshcat(self.vis, polyDataSimplified,
                                 "simplified_input")

        major_planes, plane_infos = \
            getMajorPlanes(polyDataSimplified, useVoxelGrid=False,
                           distanceToPlaneThreshold=0.005)
        coloriter = iter(plt.cm.rainbow(np.linspace(0, 1, len(major_planes))))
        scores = []
        means = []
        stds = []
        for i, plane in enumerate(major_planes):
            # Calculate this plane score, by checking is total size,
            # closeness to the camera (i.e. lowest z, since camera is at
            # origin looking up), and centrality in image
            # (i.e. closest to z axis)
            centrality_weight = 2.
            closeness_weight = 10.
            size_weight = 1.
            # Calculate mean + std in R^3
            pts = vtkNumpy.getNumpyFromVtk(plane)
            mean = np.mean(pts, axis=0)
            std = np.std(pts, axis=0)
            means.append(mean)
            stds.append(std)
            score = centrality_weight * np.linalg.norm(mean[0:2]) + \
                closeness_weight * mean[2] + \
                size_weight * 1. / max(std)
            score /= (centrality_weight + closeness_weight + size_weight)
            scores.append(score)
        scores = np.array(scores)
        scores = (scores - np.min(scores)) / (np.max(scores) - np.min(scores))

        if self.vis is not None:
            for i, plane in enumerate(major_planes):
                score_color = plt.cm.RdYlGn(1. - scores[i])
                draw_polydata_in_meshcat(self.vis, plane, "planes/%02d" % i,
                                         score_color, size=0.01)

        best_plane_i = np.argmin(scores)
        pt = plane_infos[best_plane_i][0]
        normal = plane_infos[best_plane_i][1]
        if np.dot(normal, [0., 0., 1.]) > 0.:
            normal *= -1.
        #print("pt: ", pt, " and norm: ", normal)
        #draw_plane_in_meshcat(self.vis, pt, normal,
        #                      size=[1., 1., 0.001],
        #                      name="table")

        # Todo: figure out which points are "on top of" that table cluster
        points = vtkNumpy.getNumpyFromVtk(polyData).T
        height_above_surface = np.dot(points.T - pt, normal)
        vtkNumpy.addNumpyToVtk(polyData, height_above_surface,
                               'height_above_surface')
        tabletopPoints = filterUtils.thresholdPoints(
            polyData, 'height_above_surface', [0.005, 0.05])

        distance_from_table_center = np.linalg.norm(
            vtkNumpy.getNumpyFromVtk(tabletopPoints) - means[best_plane_i],
            axis=1)
        print(distance_from_table_center)
        vtkNumpy.addNumpyToVtk(tabletopPoints, distance_from_table_center,
                               'distance_from_table_center')
        tabletopPoints = filterUtils.thresholdPoints(
            tabletopPoints, 'distance_from_table_center', [0., 0.1])

        # Finally, transform those tabletop points to point up and
        tf = transformUtils.getTransformFromOriginAndNormal(pt, normal).GetLinearInverse()
        tabletopPoints = filterUtils.transformPolyData(tabletopPoints, tf)
        draw_polydata_in_meshcat(self.vis, tabletopPoints, "tabletopPoints",
                                 color=[0., 0., 1.], size=0.001)

        #clusters = extractClusters(tabletopPoints, clusterInXY=False,
        #                           minClusterSize=100, clusterTolerance=0.01,
        #                           maxClusterSize=10000)
        #coloriter = iter(plt.cm.rainbow(np.linspace(0, 1, len(clusters))))
        #if self.vis is not None:
        #    for i, cluster in enumerate(clusters):
        #        #print("Cluster %d: " % i, cluster)
        #        draw_polydata_in_meshcat(
        #            self.vis, cluster, "clusters/%02d" % i,
        #            next(coloriter), size=0.01)


if __name__ == "__main__":
    np.set_printoptions(precision=5, suppress=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("--scans_dir",
                        type=str,
                        help="Point cloud file to segment.",
                        default="/home/gizatt/spartan/data_volume/"
                                "carrot_scans")
    parser.add_argument("--topic",
                        type=str,
                        help="Point cloud topic.",
                        default="/camera/depth_registered/points")
    parser.add_argument("--ignore_live",
                        help="Whether to use the point cloud file.",
                        default=False, action='store_true')
    args = parser.parse_args()

    if args.ignore_live is True:
        scans_dir = "/home/gizatt/spartan/data_volume/carrot_scans/"
        scans = []
        for (dir, _, files) in os.walk(scans_dir):
            for f in files:
                path = os.path.join(dir, f)
                if os.path.exists(path) and \
                   path.split("/")[-1] == "fusion_pointcloud.ply":
                    scans.append(path)

        for scan in scans:
            print("Trying scan %s" % scan)
            polyData = ioUtils.readPolyData(scan)
            segmenter = TabletopObjectSegmenter(visualize=True)
            segmenter.segment_pointcloud(polyData)
            raw_input("Enter to continue...")
    else:
        rospy.init_node("tabletop_segmenter")
        sub = rosUtils.SimpleSubscriber(
            topic=args.topic, messageType=sensor_msgs.msg.PointCloud2)
        sub.start(queue_size=1)
        segmenter = TabletopObjectSegmenter(visualize=True)
        while (1):
            print("Waiting for pc on channel %s..." % args.topic, end="")
            pc2 = sub.waitForNextMessage()
            print("... got one")
            polyData, colors = pointcloud2ToVtk(pc2)
            segmenter.segment_pointcloud(polyData)
            raw_input("Etner to continue...")
