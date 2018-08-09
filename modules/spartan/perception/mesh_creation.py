#!/usr/bin/env python

import math
import numpy as np
import os
from pydrake.all import (RotationMatrix, RollPitchYaw)
import trimesh

def create_cylinder(radius, height, sections=10):
    ''' Returns a mesh object of a cylinder, centered on the origin,
    of the specified dimensions. Long axis is z axis.
    Represents the round part with num_segments rectangular faces. '''
    return trimesh.primitives.Cylinder(
        radius=radius, height=height, sections=sections)


def create_cut_cylinder(radius, height, cutting_planes,
                        sections=10, verbose=False):
    ''' Returns a list of vertices and tris
        of a cylinder with the specified radius and height,
        cut by a list of planes (each specified in
        point-normal form as a tuple of np arrays).
        *Everything* on the positive side
        (i.e. along normal direction) will be cut off. '''
    cylinder = create_cylinder(radius, height, sections)
    box_size = 2*radius + height  # overestimate
    meshes = [cylinder]
    for (point, normal) in cutting_planes:
        # Move point as close to origin as possible
        # (so that the box doesn't clip part or none of
        # the cylinder)
        point = np.array(point)
        normal = np.array(normal)
        point = np.dot(normal, point) * normal
        box_tf = np.eye(4)

        # From https://math.stackexchange.com/questions/1956699/getting-a-transformation-matrix-from-a-normal-vector
        nx, ny, nz = normal
        if (nx**2 + ny**2 != 0.):
            nxny = np.sqrt(nx**2 + ny**2)
            box_tf[0, 0] = ny / nxny
            box_tf[0, 1] = -nx / nxny
            box_tf[1, 0] = nx*nz / nxny
            box_tf[1, 1] = ny*nz / nxny
            box_tf[1, 2] = -nxny
            box_tf[2, :3] = normal
        box_tf[:3, :3] = box_tf[:3, :3].T
        box_tf[0:3, 3] = (np.array(point) -
                          box_tf[0:3, 0:3].dot(
                            np.array([0., 0., box_size/2.])))
        meshes.append(trimesh.primitives.Box(
            extents=[box_size, box_size, box_size],
            transform=box_tf))

    if verbose:
        print "Computing difference between cylinder and %d boxes" % (
            len(cutting_planes))
    differences = trimesh.boolean.difference(meshes, 'scad')
    chull = trimesh.convex.convex_hull(differences)
    return chull