import os,sys
sys.path.insert(0, '../')

import numpy as np
from scipy import misc
from common import common
from render import render_sim
from director import vtkAll as vtk
import yaml
import time


def vtkICP(model,scene):
		icp = vtk.vtkIterativeClosestPointTransform()
		icp.SetMaximumNumberOfIterations(100)
		#need to shift centroid to center of abject after clicking on it
		icp.StartByMatchingCentroidsOn()
		icp.SetSource(model)
		icp.SetTarget(scene)
		icp.GetLandmarkTransform().SetModeToRigidBody()
		icp.Modified()
		icp.Update()
		t = vtk.vtkTransformPolyDataFilter()
		t.SetInput(model)
		t.SetTransform(icp)
		t.Update()
		transformedObject = t.GetOutput()
		print transformedObject

### enumerate tests
path  = "/home/drc/DATA/chris_labelfusion/CORL2017/logs_test/"
paths = []
for f in os.listdir(path):
	if "2017" in f:
		if "registration_result.yaml" in os.listdir(path+f):
			with open(path+f+"/registration_result.yaml") as read:
				transformYaml = yaml.load(read)
				if len(transformYaml.keys()) ==3 or True:
					paths.append((f,transformYaml.keys())) 
for i in paths:
	print i[0]

## setup rendering enviornment for mesh
view_height = 480
view_width = 640
renderer = vtk.vtkRenderer()
renderer.SetViewport(0,0,0.5,1)
renWin = vtk.vtkRenderWindow()
interactor = vtk.vtkRenderWindowInteractor()
renWin.SetSize(2*view_width,view_height)
camera = vtk.vtkCamera()
renderer.SetActiveCamera(camera);
renWin.AddRenderer(renderer);
interactor.SetRenderWindow(renWin);
common.set_up_camera_params(camera)

### setup rendering enviornment for point cloud
renderer1 = vtk.vtkRenderer()
renderer1.SetViewport(0.5,0,1,1)
camera1 = vtk.vtkCamera()
renderer1.SetActiveCamera(camera1);
renWin.AddRenderer(renderer1);
common.set_up_camera_params(camera1)

renSource = vtk.vtkRendererSource()
renSource.SetInput(renderer)
renSource.WholeWindowOff()
renSource.DepthValuesOnlyOn()
renSource.Update()

out_dir = "/home/drc/DATA/chris_labelfusion/RGBDCNN/"
object_dir = "/home/drc/DATA/chris_labelfusion/object-meshes"

samples_per_run = 1
###run through scenes
for i,j in paths[:1]:

  data_dir = path+i
  print data_dir
  data_dir_name =  os.path.basename(os.path.normpath(data_dir))
  object_dir = "/home/drc/DATA/chris_labelfusion/object-meshes"
  mesh ='meshed_scene.ply'

  #set up mesh
  actor = vtk.vtkActor()
  mapper = vtk.vtkPolyDataMapper()
  fileReader = vtk.vtkPLYReader()
  fileReader.SetFileName(data_dir+"/"+mesh)
  mapper.SetInputConnection(fileReader.GetOutputPort())
  actor.SetMapper(mapper)
  renderer.AddActor(actor)

  #add objects
  objects = common.Objects(data_dir,object_dir)
  objects.loadObjectMeshes("/registration_result.yaml",renderer1,keyword=None)


  poses = common.CameraPoses(data_dir+"/posegraph.posegraph")
  for i in np.random.choice(range(1,500),samples_per_run):
      # try:
		utimeFile = open(data_dir+"/images/"+ str(i).zfill(10) + "_utime.txt", 'r')
		utime = int(utimeFile.read())    

		#update camera transform
		cameraToCameraStart = poses.getCameraPoseAtUTime(utime)
		t = cameraToCameraStart
		common.setCameraTransform(camera, t)
		common.setCameraTransform(camera1, t)

		renSource.Update()

		# Note that the vtkPointGaussianMapper does not require vertex cells
		pc = vtk.vtkDepthImageToPointCloud()
		pc.SetInputConnection(0,renSource.GetOutputPort())
		pc.SetCamera(renderer.GetActiveCamera())
		pc.CullNearPointsOn()
		#pc.CullFarPointsOn()
		pc.ProduceVertexCellArrayOff()

		pc.Update()
		pcMapper = vtk.vtkPointGaussianMapper()
		pcMapper.SetInputConnection(pc.GetOutputPort())
		pcMapper.EmissiveOff()
		pcMapper.SetScaleFactor(0.0)

		pcActor = vtk.vtkActor()
		pcActor.SetMapper(pcMapper)

		renderer1.AddActor(pcActor)

		renWin.Render()

		scene = pcActor.GetMapper().GetInput()
		model = objects.objects.values()[0].GetMapper().GetInput()
		renderer1.RemoveActor(objects.objects.values()[0])
		vtkICP(model,scene)
		break

  #renderer.RemoveAllViewProps();
  #renderer1.RemoveAllViewProps();

  renWin.Render();
renWin.Render();
interactor.Start();

# !/usr/bin/env python
# import vtk
# from vtk.test import Testing
# from vtk.util.misc import vtkGetDataRoot
# VTK_DATA_ROOT = vtkGetDataRoot()

# # Parameters for testing
# sze = 300

# # Graphics stuff
# ren0 = vtk.vtkRenderer()
# ren0.SetViewport(0,0,0.5,1)
# ren1 = vtk.vtkRenderer()
# ren1.SetViewport(0.5,0,1,1)
# renWin = vtk.vtkRenderWindow()
# renWin.SetSize(2*sze+100,sze)
# renWin.AddRenderer(ren0)
# renWin.AddRenderer(ren1)
# iren = vtk.vtkRenderWindowInteractor()
# iren.SetRenderWindow(renWin)

# # Create pipeline, render simple object. We'll also color
# # the sphere to generate color scalars.
# sphere = vtk.vtkSphereSource()
# sphere.SetCenter(0,0,0)
# sphere.SetRadius(1)

# ele = vtk.vtkElevationFilter()
# ele.SetInputConnection(sphere.GetOutputPort())
# ele.SetLowPoint(0,-1,0)
# ele.SetHighPoint(0,1,0)

# sphereMapper = vtk.vtkPolyDataMapper()
# sphereMapper.SetInputConnection(ele.GetOutputPort())

# sphereActor = vtk.vtkActor()
# sphereActor.SetMapper(sphereMapper)

# ren0.AddActor(sphereActor)
# ren0.SetBackground(0,0,0)

# iren.Initialize()
# ren0.ResetCamera()
# ren0.GetActiveCamera().SetClippingRange(6,9)
# renWin.Render()

# # Extract rendered geometry, convert to point cloud
# # Grab just z-values
# renSource = vtk.vtkRendererSource()
# renSource.SetInput(ren0)
# renSource.WholeWindowOff()
# renSource.DepthValuesOnlyOn()
# renSource.Update()


# pc = vtk.vtkDepthImageToPointCloud()
# pc.SetInputConnection(0,renSource.GetOutputPort())
# #pc.SetInputConnection(1,renSource1.GetOutputPort())
# pc.SetCamera(ren0.GetActiveCamera())
# pc.CullNearPointsOn()
# pc.CullFarPointsOn()
# pc.ProduceVertexCellArrayOff()
# print(pc)

# timer = vtk.vtkTimerLog()
# timer.StartTimer()
# pc.Update()
# timer.StopTimer()
# time = timer.GetElapsedTime()
# print("Generate point cloud: {0}".format(time))

# pcMapper = vtk.vtkPointGaussianMapper()
# pcMapper.SetInputConnection(pc.GetOutputPort())
# pcMapper.EmissiveOff()
# pcMapper.SetScaleFactor(0.0)

# pcActor = vtk.vtkActor()
# pcActor.SetMapper(pcMapper)

# ren1.AddActor(pcActor)
# ren1.SetBackground(0,0,0)
# cam = ren1.GetActiveCamera()
# cam.SetFocalPoint(0,0,0)
# cam.SetPosition(1,1,1)
# ren1.ResetCamera()

# renWin.Render()
# iren.Start()


