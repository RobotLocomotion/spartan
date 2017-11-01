#!/usr/bin/env python

# This example shows how to construct a surface from a point cloud.
# First we generate a volume using the
# vtkSurfaceReconstructionFilter. The volume values are a distance
# field. Once this is generated, the volume is countoured at a
# distance value of 0.0.

import os
import string
import vtk
from director import visualization as vis
from director import filterUtils
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

poly = om.findObjectByName("reconstructed_pointcloud.vtp").polyData
# Construct the surface and create isosurface.

surf = vtk.vtkSurfaceReconstructionFilter()
surf.SetInputData(poly)
surf.Update()

cf = vtk.vtkContourFilter()
cf.SetInputData(surf.GetOutput())
cf.SetValue(0, 0.0)
cf.Update()

# Sometimes the contouring algorithm can create a volume whose gradient
# vector and ordering of polygon (using the right hand rule) are
# inconsistent. vtkReverseSense cures this problem.
reverse = vtk.vtkReverseSense()
reverse.SetInputData(cf.GetOutput())
reverse.ReverseCellsOn()
reverse.ReverseNormalsOn()
reverse.Update()

vis.showPolyData(reverse.GetOutput(),"pointcloud_mesh",color = [1,0,0])