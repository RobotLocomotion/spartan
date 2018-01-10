from director import mainwindowapp
from director import robotsystem
from director import applogic as app
from director import consoleapp
from director import transformUtils
from director import mainwindowapp
from director import filterUtils
from director import depthscanner
from director import ioUtils
from director import visualization as vis
import numpy as np
from director import objectmodel as om
from director import vtkAll as vtk
import Image
from common import common
import os

def render_depth(data_dir,data_dir_name,num_im,mesh,out_dir,view_height=640,view_width=480):
  actor = vtk.vtkActor()
  renderer = vtk.vtkRenderer()
  renWin = vtk.vtkRenderWindow()
  interactor = vtk.vtkRenderWindowInteractor()
  filter1= vtk.vtkWindowToImageFilter()
  imageWriter = vtk.vtkPNGWriter()
  scale =vtk.vtkImageShiftScale()
  renWin.SetSize(view_height,view_width)
  camera = vtk.vtkCamera()
  renderer.SetActiveCamera(camera);
  renWin.AddRenderer(renderer);
  interactor.SetRenderWindow(renWin);
  #setup camera calibration
  common.set_up_camera_params(camera)

  if use_mesh: #use meshed version of scene
    mapper = vtk.vtkPolyDataMapper()
    fileReader = vtk.vtkPLYReader()
    fileReader.SetFileName(sys.argv[1]+"/"+sys.argv[3])
    mapper.SetInputConnection(fileReader.GetOutputPort())
    actor.SetMapper(mapper)
    renderer.AddActor(actor)
  else: #import just the objects
    objects = common.Objects(data_dir,"/home/drc/spartan/Data_objects")
    objects.loadObjectMeshes("/registration_result.yaml",renderer,keyword="drill")


  #setup filters
  filter1.SetInput(renWin)
  filter1.SetMagnification(1)
  filter1.SetInputBufferTypeToZBuffer()
  windowToColorBuffer = vtk.vtkWindowToImageFilter()
  windowToColorBuffer.SetInput(renWin)
  windowToColorBuffer.SetInputBufferTypeToRGB()     
  scale.SetOutputScalarTypeToUnsignedShort()
  scale.SetScale(1000);

  poses = common.CameraPoses(data_dir+"/posegraph.posegraph")
  for i in range(1,num_im+1):
      print "rendering image "+str(i)
      utimeFile = open(data_dir+"/images/"+ str(i).zfill(10) + "_utime.txt", 'r')
      utime = int(utimeFile.read())    

      #update camera transform
      cameraToCameraStart = poses.getCameraPoseAtUTime(utime)
      t = cameraToCameraStart
      common.setCameraTransform(camera, t)
      renWin.Render()

      #update filters
      filter1.Modified()
      filter1.Update()
      windowToColorBuffer.Modified()
      windowToColorBuffer.Update()

      #extract depth image
      depthImage = vtk.vtkImageData()
      pts = vtk.vtkPoints()
      ptColors = vtk.vtkUnsignedCharArray()
      vtk.vtkDepthImageUtils.DepthBufferToDepthImage(filter1.GetOutput(), windowToColorBuffer.GetOutput(), camera, depthImage, pts, ptColors)
      scale.SetInputData(depthImage)
      scale.Update()

      #write out depth image
      imageWriter.SetFileName(out_dir+str(i).zfill(10)+"_"+data_dir_name+"_depth_ground_truth.png");
      imageWriter.SetInputConnection(scale.GetOutputPort());
      imageWriter.Write();
  
  renWin.Render();
  interactor.Start();

if __name__ == '__main__':
  #setup
  view_height = 640
  view_width = 480
  data_dir = sys.argv[1]
  data_dir_name =  os.path.basename(os.path.normpath(data_dir))
  num_im = int(sys.argv[2])
  mesh = sys.argv[3]
  out_dir = sys.argv[1]+"/images/" if len(sys.argv)==3 else sys.argv[4]

  print "rendering Label Fusion data", data_dir_name
  use_mesh = False

  #setup rendering enviornment
  actor = vtk.vtkActor()
  renderer = vtk.vtkRenderer()
  renWin = vtk.vtkRenderWindow()
  interactor = vtk.vtkRenderWindowInteractor()
  filter1= vtk.vtkWindowToImageFilter()
  imageWriter = vtk.vtkPNGWriter()
  scale =vtk.vtkImageShiftScale()
  renWin.SetSize(view_height,view_width)
  camera = vtk.vtkCamera()
  renderer.SetActiveCamera(camera);
  renWin.AddRenderer(renderer);
  interactor.SetRenderWindow(renWin);
  #setup camera calibration
  common.set_up_camera_params(camera)

  if use_mesh: #use meshed version of scene
    mapper = vtk.vtkPolyDataMapper()
    fileReader = vtk.vtkPLYReader()
    fileReader.SetFileName(sys.argv[1]+"/"+sys.argv[3])
    mapper.SetInputConnection(fileReader.GetOutputPort())
    actor.SetMapper(mapper)
    renderer.AddActor(actor)
  else: #import just the objects
    objects = common.Objects(data_dir,"/home/drc/spartan/Data_objects")
    objects.loadObjectMeshes("/registration_result.yaml",renderer,keyword="drill")


  #setup filters
  filter1.SetInput(renWin)
  filter1.SetMagnification(1)
  filter1.SetInputBufferTypeToZBuffer()
  windowToColorBuffer = vtk.vtkWindowToImageFilter()
  windowToColorBuffer.SetInput(renWin)
  windowToColorBuffer.SetInputBufferTypeToRGB()     
  scale.SetOutputScalarTypeToUnsignedShort()
  scale.SetScale(1000);

  poses = common.CameraPoses(data_dir+"/posegraph.posegraph")
  for i in range(1,num_im+1):
      print "rendering image "+str(i)
      utimeFile = open(data_dir+"/images/"+ str(i).zfill(10) + "_utime.txt", 'r')
      utime = int(utimeFile.read())    

      #update camera transform
      cameraToCameraStart = poses.getCameraPoseAtUTime(utime)
      t = cameraToCameraStart
      common.setCameraTransform(camera, t)
      renWin.Render()

      #update filters
      filter1.Modified()
      filter1.Update()
      windowToColorBuffer.Modified()
      windowToColorBuffer.Update()

      #extract depth image
      depthImage = vtk.vtkImageData()
      pts = vtk.vtkPoints()
      ptColors = vtk.vtkUnsignedCharArray()
      vtk.vtkDepthImageUtils.DepthBufferToDepthImage(filter1.GetOutput(), windowToColorBuffer.GetOutput(), camera, depthImage, pts, ptColors)
      scale.SetInputData(depthImage)
      scale.Update()

      #write out depth image
      imageWriter.SetFileName(out_dir+str(i).zfill(10)+"_"+data_dir_name+"_depth_ground_truth.png");
      imageWriter.SetInputConnection(scale.GetOutputPort());
      #imageWriter.Write();
  
  renderer.RemoveAllViewProps();
  renWin.Render();
  interactor.Start();#begiining