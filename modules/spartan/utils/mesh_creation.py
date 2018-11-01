#!/usr/bin/env python

import math
import numpy as np
import os
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


def export_sdf(mesh, name, directory, color=[0.75, 0.2, 0.2, 1.],
               mass=None, moment=None):
    sdf_template = open("sdf_template.xml").read()
    centroid_string = "%f %f %f 0. 0. 0." % (
        mesh.center_mass[0], mesh.center_mass[1], mesh.center_mass[2])
    if mass is None:
        mass = mesh.mass
    if moment is None:
        moment = mesh.moment_inertia
    color_string = " ".join(str(c) for c in color)
    mesh_path = "%s/meshes/%s.obj" % (directory, name)
    sdf_string = sdf_template.format(
        name=name, centroid_string=centroid_string, mass=mass,
        color_string=color_string, mesh_path=mesh_path,
        ixx=moment[0][0], ixy=moment[0][1], ixz=moment[0][2],
        iyy=moment[1][1], iyz=moment[1][2], izz=moment[2][2])

    os.system("mkdir -p %s/meshes" % directory)

    with open(mesh_path, 'w') as f:
        f.write(trimesh.io.wavefront.export_wavefront(mesh))
    with open("%s/%s.sdf" % (directory, name), 'w') as f:
        f.write(sdf_string)


if __name__ == "__main__":
    cyl = create_cut_cylinder(radius=3, height=5,
                              cutting_planes=[([0., 0, 0], [0.7, 0.0, 0.7])],
                              sections=10, verbose=True)
    export_sdf(cyl, "test_mesh", "/tmp/test_mesh_export/")
    print open("/tmp/test_mesh_export/test_mesh.sdf").read()

    loaded = trimesh.io.load.load(
        open("/tmp/test_mesh_export/meshes/test_mesh.obj"),
        file_type="obj")
    loaded.show()
