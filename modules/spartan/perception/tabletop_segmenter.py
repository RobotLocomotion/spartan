#!/usr/bin/env python
from __future__ import print_function

import argparse
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

from director import filterUtils
from director import ioUtils
from director.shallowCopy import shallowCopy
from director import vtkNumpy
from director import vtkAll as vtk

import meshcat
import meshcat.geometry as meshcat_g
import meshcat.transformations as meshcat_tf



def applyEuclideanClustering(dataObj, clusterTolerance=0.05, minClusterSize=100, maxClusterSize=1e6):
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

    if (clusterInXY == True):
        ''' If Points are seperated in X&Y, then cluster outside this '''
        polyDataXY = vtk.vtkPolyData()
        polyDataXY.DeepCopy(polyData)
        points=vtkNumpy.getNumpyFromVtk(polyDataXY , 'Points') # shared memory
        points[:,2] = 0.0
        #showPolyData(polyDataXY, 'polyDataXY', visible=False, parent=getDebugFolder())
        polyDataXY = applyEuclideanClustering(polyDataXY, **kwargs)
        clusterLabels = vtkNumpy.getNumpyFromVtk(polyDataXY, 'cluster_labels')
        vtkNumpy.addNumpyToVtk(polyData, clusterLabels, 'cluster_labels')

    else:
        polyData = applyEuclideanClustering(polyData, **kwargs)
        clusterLabels = vtkNumpy.getNumpyFromVtk(polyData, 'cluster_labels')


    clusters = []
    for i in xrange(1, clusterLabels.max() + 1):
        cluster = filterUtils.thresholdPoints(polyData, 'cluster_labels', [i, i])
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
        else:
            break

    return polyDataList


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


class TabletopObjectSegmenter:
    def __init__(self,
                 zmq_url="tcp://127.0.0.1:6000"):

        print("Opening meshcat vis... will hang if no server"
              "\tis running.")
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        self.vis["perception"].delete()
        self.vis["perception/tabletopsegmenter"].delete()

    def segment_pointcloud(self, polyData):
        polyDataSimplified = applyVoxelGrid(polyData, leafSize=0.005)
        draw_polydata_in_meshcat(self.vis, polyDataSimplified,
                                 "simplified_input")

        points = vtkNumpy.getNumpyFromVtk(polyDataSimplified).T
        vtkNumpy.addNumpyToVtk(polyDataSimplified, points[2, :], 'z_height')
        tabletopPoints = filterUtils.thresholdPoints(
            polyDataSimplified, 'z_height', [-0.001, 100.])

        clusters = extractClusters(tabletopPoints, clusterInXY=False)
        coloriter = iter(plt.cm.rainbow(np.linspace(0, 1, len(clusters))))
        for i, cluster in enumerate(clusters):
            print("Cluster %d: " % i, cluster)
            draw_polydata_in_meshcat(self.vis, cluster, "clusters/%02d" % i,
                                     next(coloriter), size=0.01)





if __name__ == "__main__":
    np.set_printoptions(precision=5, suppress=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_file",
                        type=str,
                        help="Point cloud file to segment.",
                        default="/home/gizatt/spartan/data_volume/"
                                "carrot_scans/2018-07-10-21-32-49/"
                                "processed/fusion_pointcloud.ply")
    args = parser.parse_args()

    scans_dir = "/home/gizatt/spartan/data_volume/carrot_scans/"
    scans = []
    for (dir, _, files) in os.walk(scans_dir):
        for f in files:
            path = os.path.join(dir, f)
            if os.path.exists(path) and \
               path.split("/")[-1] == "fusion_pointcloud.ply":
                scans.append(path)

    for scan in scans:
        polyData = ioUtils.readPolyData(scan)
        segmenter = TabletopObjectSegmenter()
        segmenter.segment_pointcloud(polyData)
        raw_input("Enter to continue...")
