#!/usr/bin/env python
from __future__ import print_function

import argparse
import cv2
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
import scipy.spatial
import os
import sys
import matplotlib.pyplot as plt

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

import octomap


def do_pointcloud_preprocessing(pc2):
    pc_np = ros_numpy.numpify(pc2)

    # First, back out the depth image and get a mask
    # for where we have no returns
    nan_mask = np.logical_not(np.isfinite(pc_np['z']))
    nan_mask = np.stack([nan_mask, nan_mask, nan_mask], axis=-1)

    xyz_im = np.stack([pc_np['x'], pc_np['y'], pc_np['z']], axis=-1)
    xyz_im[nan_mask] = 30.
    xyz_im_avg = xyz_im.copy()
    for k in range(10):
        for l in range(3):
            xyz_im_avg[:, :, l] = cv2.blur(xyz_im_avg[:, :, l], (5, 5))
        xyz_im_avg = np.where(nan_mask, xyz_im_avg, xyz_im)
    ksize = 21
    normal_xx = cv2.Sobel(xyz_im_avg[:, :, 0], cv2.CV_32F, 1, 0, ksize=ksize)
    normal_xy = cv2.Sobel(xyz_im_avg[:, :, 0], cv2.CV_32F, 0, 1, ksize=ksize)
    normal_yx = cv2.Sobel(xyz_im_avg[:, :, 1], cv2.CV_32F, 1, 0, ksize=ksize)
    normal_yy = cv2.Sobel(xyz_im_avg[:, :, 1], cv2.CV_32F, 0, 1, ksize=ksize)
    normal_zx = cv2.Sobel(xyz_im_avg[:, :, 2], cv2.CV_32F, 1, 0, ksize=ksize)
    normal_zy = cv2.Sobel(xyz_im_avg[:, :, 2], cv2.CV_32F, 0, 1, ksize=ksize)

    x_normals = np.stack([normal_xx, normal_yx, normal_zx], axis=-1)
    y_normals = np.stack([normal_xy, normal_yy, normal_zy], axis=-1)
    normals = -np.cross(x_normals, y_normals)
    ln = np.linalg.norm(normals, axis=-1)
    normals = normals / np.stack([ln, ln, ln], axis=-1)

    # Also compute curvatures
    curvatures = np.zeros(normals.shape[:2])
    steps = range(5, 75, 5)
    for step in steps:
        diff = xyz_im_avg[step:, :, :] - xyz_im_avg[:-step, :, :]
        normal_diff = normals[step:, :, :] - normals[:-step, :, :]
        est = (np.sum(normal_diff * diff, axis=-1)
               / (np.linalg.norm(diff, axis=-1)**2))
        curvatures[step:, :] += est
        curvatures[:-step, :] += est

        diff = xyz_im_avg[:, step:, :] - xyz_im_avg[:, :-step, :]
        normal_diff = normals[:, step:, :] - normals[:, :-step, :]
        est = (np.sum(normal_diff * diff, axis=-1)
               / (np.linalg.norm(diff, axis=-1)**2))
        curvatures[:, step:] += est
        curvatures[:, :-step] += est

    curvatures /= len(steps)
    #curvatures[np.logical_not(np.isfinite(curvatures))] = 0.
    #curvatures = np.clip(curvatures, 0., 100.)
    print(np.min(curvatures), np.max(curvatures), curvatures)

    # Reject stuff with normals too far from the camera depth axis
    _, reject_im = cv2.threshold(np.dot(normals, [0., 0., -1.]),
                                 0.2, 1., cv2.THRESH_BINARY)
    reject_im = reject_im.astype(bool)
    reject_im = np.stack([reject_im, reject_im, reject_im], axis=-1)

    points = np.stack([pc_np['x'], pc_np['y'], pc_np['z']], axis=-1)
    points = np.where(reject_im, points, np.nan)
    normals = np.where(reject_im, normals, np.nan)

    fig = plt.gcf()
    plt.clf()
    #plt.subplot(3, 1, 1)
    #plt.imshow(xyz_im_avg[:, :, 2])
    #plt.subplot(3, 1, 2)
    #plt.imshow(normals)
    #plt.subplot(3, 1, 3)
    ax = plt.subplot(2, 1, 1)
    im = plt.imshow(np.abs(curvatures), vmin=0., vmax=100.)
    fig.colorbar(im, ax=ax)
    ax = plt.subplot(2, 1, 2)
    im = plt.imshow((np.abs(curvatures) > 30.).astype(float))
    fig.colorbar(im, ax=ax)


    plt.pause(1E-6)

    return points, normals, curvatures


def flatten_3d_image_consistently(im):
    # reshape is a little sketchy because of memory ordering questions
    return np.stack((im[:, :, 0].flatten(),
                     im[:, :, 1].flatten(),
                     im[:, :, 2].flatten()), axis=-1)


def convert_pc_np_to_vtk(points, normals, curvatures):
    n_points = points.shape[0]*points.shape[1]
    points_flat = flatten_3d_image_consistently(points)
    normals_flat = flatten_3d_image_consistently(normals)
    curvatures_flat = curvatures.flatten()
    good_entries = np.logical_not(np.isnan(points_flat, ).any(axis=1))
    points_flat = points_flat[good_entries, :]
    normals_flat = normals_flat[good_entries, :]
    curvatures_flat = curvatures_flat[good_entries]
    pc_vtk = vtkNumpy.numpyToPolyData(
        points_flat, pointData=None, createVertexCells=True)
    vtkNumpy.addNumpyToVtk(pc_vtk, normals_flat, "Normals")
    vtkNumpy.addNumpyToVtk(pc_vtk, curvatures_flat, "Curvatures")
    return pc_vtk


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


def draw_polydata_in_meshcat(vis, polyData, name, color=None, size=0.001,
                             with_normals=False, color_by_curvature=False):
    points = vtkNumpy.getNumpyFromVtk(polyData).T
    if color_by_curvature:
        curv = vtkNumpy.getNumpyFromVtk(polyData, "Curvatures")
        curv -= np.min(curv)
        curv /= np.max(curv)
        colors = plt.cm.rainbow(curv)
    elif color is not None:
        colors = np.tile(color[0:3], [points.shape[1], 1]).T
    else:
        colors = np.zeros(points.shape) + 1.
    vis["perception"]["tabletopsegmenter"][name].set_object(
        meshcat_g.PointCloud(position=points, color=colors, size=size))
    if with_normals:
        normals = vtkNumpy.getNumpyFromVtk(polyData, "Normals").T
        colors = np.tile([1., 0., 0.], [points.shape[1], 1]).T
        segments = np.zeros((3, normals.shape[1]*2))
        for k in range(normals.shape[1]):
            segments[:, 2*k] = points[:, k]
            segments[:, 2*k+1] = points[:, k] + normals[:, k]*0.05
        vis["perception"]["tabletopsegmenter"][name]["normals"].set_object(
            meshcat_g.LineSegments(position=segments,
                                   color=colors, linewidth=size/2.))


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

    def get_table_surface_plane_info(self, polyData):
        major_planes, plane_infos = \
            getMajorPlanes(polyData, useVoxelGrid=False,
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
        # Change pt to be at the table center
        pt = means[best_plane_i] - (means[best_plane_i] - pt).dot(normal)
        return pt, normal

    def segment_pointcloud(self, polyData, plane_info=None):
        polyDataSimplified = applyVoxelGrid(polyData, leafSize=0.005)
        if self.vis is not None:
            draw_polydata_in_meshcat(self.vis, polyDataSimplified,
                                     "simplified_input")
        if plane_info is None:
            pt, normal = self.get_table_surface_plane_info(polyDataSimplified)
        else:
            pt, normal = plane_info
        # Todo: figure out which points are "on top of" that table cluster
        points = vtkNumpy.getNumpyFromVtk(polyData).T
        height_above_surface = np.dot(points.T - pt, normal)
        vtkNumpy.addNumpyToVtk(polyData, height_above_surface,
                               'height_above_surface')
        tabletopPoints = filterUtils.thresholdPoints(
            polyData, 'height_above_surface', [0.005, 0.05])

        distance_from_table_center = np.linalg.norm(
            vtkNumpy.getNumpyFromVtk(tabletopPoints) - pt, axis=1)
        vtkNumpy.addNumpyToVtk(tabletopPoints, distance_from_table_center,
                               'distance_from_table_center')
        tabletopPoints = filterUtils.thresholdPoints(
            tabletopPoints, 'distance_from_table_center', [0., 0.1])

        # tabletopPoints = applyVoxelGrid(tabletopPoints, leafSize=0.001)
        # Finally, transform those tabletop points to point up
        tf = transformUtils.getTransformFromOriginAndNormal(
            pt, normal).GetLinearInverse()
        tabletopPoints = filterUtils.transformPolyData(tabletopPoints, tf)
        old_normals = vtkNumpy.getNumpyFromVtk(tabletopPoints, "Normals")
        vtkNumpy.addNumpyToVtk(
            tabletopPoints,
            transformUtils.getNumpyFromTransform(tf)[0:3, 0:3].dot(
                old_normals.T).T,
            "Normals")
        draw_polydata_in_meshcat(self.vis, tabletopPoints, "tabletopPoints",
                                 color=[0., 0., 1.], size=0.001,
                                 with_normals=False,
                                 color_by_curvature=False)

        return tabletopPoints, (pt, normal)

    def fuse_polydata_list_with_octomap(self, all_pds):
        cell_size = 0.002
        tree = octomap.OcTree(cell_size)
        tree.setOccupancyThres(0.9)
        print("Prob hit: %f, prob miss: %f" %
              (tree.getProbHit(), tree.getProbMiss()))
        print("Occupancy thresh: %f" % (tree.getOccupancyThres()))
        for pd in all_pds:
            tree.insertPointCloud(
                vtkNumpy.getNumpyFromVtk(pd).astype(np.float64),
                np.zeros(3))
        output_cloud = np.zeros((tree.size(), 3))
        output_color = np.zeros((tree.size(), 4))
        k = 0
        for node in tree.begin_tree():
            if node.isLeaf() and tree.isNodeOccupied(node):
                output_cloud[k, :] = np.array(node.getCoordinate())
                output_color[k, :] = plt.cm.rainbow(node.getOccupancy())
                k += 1
        output_cloud.resize((k, 3))
        output_color.resize((k, 4))

        if self.vis is not None:
            self.vis["perception"]["tabletopsegmenter"]["fused_cloud"]\
                .set_object(meshcat_g.PointCloud(position=output_cloud.T,
                                                 color=output_color.T,
                                                 size=cell_size))
        return vtkNumpy.numpyToPolyData(output_cloud)

    def find_flippable_planes(self, polyData, n_seeds=100,
                              max_plane_size=0.02,
                              plane_inlier_tolerance=0.005,
                              normal_alignment_threshold=0.2):
        draw_polydata_in_meshcat(self.vis, polyData, "tabletopPoints",
                                 color=[0., 0., 1.], size=0.001,
                                 with_normals=False,
                                 color_by_curvature=False)
        coherent_planes = []
        normals = vtkNumpy.getNumpyFromVtk(polyData, "Normals").T
        points = vtkNumpy.getNumpyFromVtk(polyData).T

        # Build a KD tree from the points
        kdtree = sp.spatial.KDTree(points.T)

        points_accounted_for = np.zeros(points.shape[1]).astype(bool)
        scores = np.zeros(n_seeds)
        point_sets = []
        normal_sets = []
        for k in range(n_seeds):
            # pick a random points to be on-plane and not-on-plane
            ind = np.random.randint(points.shape[1])
            while points_accounted_for[ind] is True:
                ind = np.random.randint(points.shape[1])
            point = points[:, ind]
            normal = normals[:, ind]

            fail = False
            for fit_iter in range(3):
                # find all points within the max plane size of it
                nearby_inds = kdtree.query_ball_point(point, max_plane_size)
                if len(nearby_inds) == 0:
                    fail = True
                    break
                nearby_pts = points[:, nearby_inds]
                nearby_normals = normals[:, nearby_inds]
                # reject points that are too far from the source point's plane
                distances_to_plane = np.abs((nearby_pts.T - point).dot(normal))
                inlier_inds = distances_to_plane < plane_inlier_tolerance
                # And reject points whose normals are too far from  the source
                # point's normal
                normal_distance = 1. - np.abs(nearby_normals.T.dot(normal))
                inlier_inds *= normal_distance < normal_alignment_threshold

                inlier_points = nearby_pts[:, inlier_inds]
                inlier_normals = nearby_normals[:, inlier_inds]

                if np.sum(inlier_inds) == 0:
                    fail = True
                    break

                point = np.mean(inlier_points, axis=1)
                plane_fit_points = inlier_points.T - point
                _, _, V = np.linalg.svd(plane_fit_points)
                normal = V[-1]

            if fail:
                scores[k] = -np.inf
                point_sets.append(np.zeros([3, 0]))
                normal_sets.append(np.zeros([3, 0]))
            else:
                normal_score = -1.0*np.sum(normal_distance[inlier_inds]) / normal_alignment_threshold / nearby_pts.shape[1]
                plane_score = -1.0*np.sum(distances_to_plane[inlier_inds]) / nearby_pts.shape[1]
                in_plane_points = \
                    inlier_points - \
                    (np.dot(inlier_points.T - point, normal) *
                        (inlier_points.T - point).T)
                size_score = np.max(
                    np.linalg.norm(in_plane_points.T - point, axis=0))
                scores[k] = normal_score + plane_score + size_score
                print("Normal: %f, Plane: %f, Size: %f" % (normal_score, plane_score, size_score))
                point_sets.append(inlier_points)
                normal_sets.append(inlier_normals)
                points_accounted_for[nearby_inds][inlier_inds] = True

        if self.vis is not None:
            n_to_vis = 100
            best_run_k = np.argsort(scores)[-n_to_vis:]
            colors = iter(plt.cm.rainbow(np.linspace(0, 1, n_to_vis)))
            self.vis["perception"]["tabletopsegmenter"]["flippable"].delete()
            for k in best_run_k:
                print("Num %d: Score %f with %d points" %
                      (k, scores[k], point_sets[k].shape[1]))
                color = np.tile(next(colors), [point_sets[k].shape[1], 1]).T
                print("Color shape: ", color.shape)
                self.vis["perception"]["tabletopsegmenter"]["flippable"]\
                    ["%d" % k].set_object(
                        meshcat_g.PointCloud(position=point_sets[k],
                                             color=color,
                                             size=0.001))


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
        scans_dir = "/home/gizatt/spartan/data_volume/"\
                    "carrot_scans/2018-08-29-real-carrots"
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
        plt.figure()
        segmenter = TabletopObjectSegmenter(visualize=True)
        while (1):
            print("Waiting for pc on channel %s..." % args.topic, end="")
            all_pds = []
            plane_info = None
            for k in range(3):
                pc2 = sub.waitForNextMessage()
                print("Processing scan...")
                points, normals, curvatures = do_pointcloud_preprocessing(pc2)
                polyData = convert_pc_np_to_vtk(points, normals, curvatures)
                pd, plane_info = segmenter.segment_pointcloud(
                    polyData, plane_info=plane_info)
                all_pds.append(pd)

            #fused_polydata = segmenter.fuse_polydata_list_with_octomap(all_pds)
            all_pd = filterUtils.appendPolyData(all_pds)
            candidate_flip_surfaces = segmenter.find_flippable_planes(all_pd)
            np.save("cloud.npy", vtkNumpy.getNumpyFromVtk(pd))
            raw_input("Etner to continue...")
