import functools
import math
import numpy as np
import scipy as sp
import time
import trimesh
import sys

from mesh_creation import create_cut_cylinder

from pydrake.all import (
    AutoDiffXd,
    MathematicalProgram,
    SolutionResult)
import pydrake.math as drake_math
import pydrake.forwarddiff as forwarddiff

import meshcat
import meshcat.geometry as meshcat_g
import meshcat.transformations as meshcat_tf

vis = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
vis["perception"]["fit"].delete()

points = np.load("cloud.npy")
print("Loaded %d points from cloud.npy" % points.shape[0])
colors = points.copy()
colors -= np.min(colors, axis=0)
colors /= np.max(colors, axis=0)
vis["perception"]["fit"]["points"].set_object(
    meshcat_g.PointCloud(position=points.T,
                         color=colors.T,
                         size=0.001))


def vectorized_ad_helper(function, x):
    x_ad = np.empty(x.shape, dtype=AutoDiffXd)
    for i in range(x.size):
        der = np.zeros(x.size)
        der[i] = 1
        x_ad.flat[i] = AutoDiffXd(x.flat[i], der)
    y_ad = function(x_ad)
    return y_ad


def meshcat_draw_line_with_dots(path, x1, x2, N=50, size=0.0001,
                                color=[1., 1., 1.]):
    color = np.tile(np.array(color), [N, 1]).T
    pts = np.zeros([N, 3])
    for i, p in enumerate(np.linspace(0., 1., N)):
        pts[i, :] = x1 * p + x2 * (1. - p)
    vis[path].set_object(meshcat_g.PointCloud(
        position=pts.T,
        color=color,
        size=size))


class DeformableCarrot():
    def __init__(self):
        self.trimesh = create_cut_cylinder(
            radius=0.02, height=0.02,
            cutting_planes=[([0, 0, 0], [0, 1, 0])],
            sections=10)
        self.n_verts = self.trimesh.vertices.shape[0]
        self.n_faces = self.trimesh.faces.shape[0]
        self.deformation_map = []
        self.tf = np.eye(4)

    def compute_alignment_error(self, points, R, T):
        pt_tf = R.dot(points.T).T + T
        if isinstance(pt_tf[0, 0], AutoDiffXd):
            pt_tf_val = np.array([[x.value() for x in y] for y in pt_tf])
        else:
            pt_tf_val = pt_tf
        _, _, faces = self.trimesh.nearest.on_surface(pt_tf_val)
        total_error = 0.
        for i in range(pt_tf.shape[0]):
            face_elems = self.trimesh.faces[faces[i], :]
            xi = [self.trimesh.vertices[v] for v in face_elems]
            normal = np.cross(xi[2] - xi[0], xi[1] - xi[0])
            pt = pt_tf[i, :]
            total_error += np.linalg.norm((pt - xi[0]).dot(normal))
        return total_error

    def align(self, points, iterations=20):
        for k in range(iterations):
            R = self.tf[:3, :3].T
            T = -R.dot(self.tf[:3, 3])
            pts_tf = R.dot(points.T).T + T

            # As a speed hack, do a broadphase collision check first
            distances_broadphase = np.linalg.norm(pts_tf, axis=1)
            thresh = 0.05
            selector_broadphase = distances_broadphase <= thresh
            while np.sum(selector_broadphase) == 0:
                thresh *= 2.
                selector_broadphase = distances_broadphase <= thresh
            # Only bother with those within a pretty close distance
            pts_tf = pts_tf[selector_broadphase]
            # And then further randomly downselect
            np.random.shuffle(pts_tf)
            pts_tf = pts_tf[0:min(400, pts_tf.shape[0]), :]
            vis["perception"]["fit"]["points_selected"].set_object(
                meshcat_g.PointCloud(
                    position=(self.tf[:3, :3].dot(pts_tf.T).T +
                              self.tf[:3, 3]).T,
                    color=pts_tf.T,
                    size=0.001))

            closest, distance, faces = self.trimesh.nearest.on_surface(pts_tf)

            # Align a with points that are within a threshold distance
            selection = np.argsort(distance)[:pts_tf.shape[0] / 2]
            closest = closest[selection]
            pts_tf_closest = pts_tf[selection, :]
            #for i, (x1, x2) in enumerate(zip(closest, pts_tf_closest)):
            #    meshcat_draw_line_with_dots(
            #        "perception/fit/corresp/%d" % i,
            #        self.tf[:3, :3].dot(x1)+self.tf[:3, 3],
            #        self.tf[:3, :3].dot(x2)+self.tf[:3, 3])

            matrix, _, err = trimesh.registration.procrustes(
                pts_tf_closest, closest, reflection=False, scale=False)

            self.tf = np.dot(self.tf, np.linalg.inv(matrix))

        return err

    def deform(self, points):
        for k in range(iterations):
            R = self.tf[:3, :3].T
            T = -R.dot(self.tf[:3, 3])

            def error_wrapper(x):
                R = x[0:9].reshape(3, 3)
                T = x[9:12]
                return self.compute_alignment_error(points, R, T)
            error = vectorized_ad_helper(error_wrapper,
                                         np.hstack([R.ravel(), T]))
            print "Alignment error:"
            print "\t value: ", error.value()
            R_deriv = error.derivatives()[0:9].reshape(3, 3)
            T_deriv = error.derivatives()[9:12]
            print "\t R deriv: ", R_deriv
            print "\t T deriv: ", T_deriv

            R -= R_deriv * 1.
            T -= T_deriv * 0.01
            u, _, v = np.linalg.svd(R)
            R = u.dot(v)

        self.tf[:3, :3] = R.T
        self.tf[:3, 3] = -R.T.dot(T)



    def draw(self, vis):
        vis["perception"]["fit"]["carrot"].set_object(
            meshcat_g.ObjMeshGeometry(
                contents=trimesh.io.wavefront.export_wavefront(self.trimesh)))
        vis["perception"]["fit"]["carrot"].set_transform(self.tf)


for random_seed in range(10):
    np.random.seed(random_seed)
    carrot = DeformableCarrot()
    carrot.tf[0:3, 3] = np.mean(points, axis=0) + \
        np.random.random(3)*0.1 - np.ones(3)*0.05
    carrot.draw(vis)
    tf_est = carrot.tf.copy()
    for k in range(500):
        tf_est = tf_est*0.9 + carrot.tf*0.1
        err = carrot.align(points, iterations=1)
        tf_after = carrot.tf.copy()
        ang_diff = np.linalg.norm(
                sp.linalg.logm(
                    tf_est[0:3, 0:3].T.dot(tf_after[0:3, 0:3])), 'fro')
        trans_diff = np.linalg.norm(tf_est[0:3, 3] - tf_after[0:3, 3])
        diff = ang_diff + trans_diff
        carrot.draw(vis)
        print "Diff: Ang %f, Trans %f, Total %f" % (ang_diff, trans_diff, diff)
        if ang_diff < 0.05 and trans_diff < 0.001:
            break
