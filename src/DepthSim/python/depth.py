from director import mainwindowapp
from director import robotsystem
from director import applogic as app
from director import consoleapp
from director import transformUtils
from director import mainwindowapp
from director import depthscanner
from director import ioUtils
from director import visualization as vis
import numpy as np
from director import objectmodel as om
from director import vtkAll as vtk
import Image
import common as common

if __name__ == '__main__':
  #setup
  view_height = 640
  view_width = 480
  data_dir = sys.argv[1]
  num_im = int(sys.argv[2])
  mesh = sys.argv[3]

  #setup rendering enviornment
  mapper =vtk.vtkPolyDataMapper()
  actor =vtk.vtkActor()
  renderer =vtk.vtkRenderer()
  renWin =vtk.vtkRenderWindow()
  interactor = vtk.vtkRenderWindowInteractor()
  fileReader = vtk.vtkPLYReader()
  filter1= vtk.vtkWindowToImageFilter()
  imageWriter = vtk.vtkPNGWriter()
  scale =vtk.vtkImageShiftScale()
  fileReader.SetFileName(sys.argv[1]+"/"+sys.argv[3])
  renWin.SetSize(view_height,view_width)
  camera = vtk.vtkCamera()
  renderer.SetActiveCamera(camera);
  mapper.SetInputConnection(fileReader.GetOutputPort());
  actor.SetMapper(mapper);
  renderer.AddActor(actor);
  renWin.AddRenderer(renderer);
  interactor.SetRenderWindow(renWin);
  
  #setup camera calibration
  common.set_up_camera_params(camera)

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

      #encode normals into file
      #vtk_normal_image = encode_normal_rgb(view,view_height,view_width)
      #print "rendered normal image at " + str(utime)

      #write out depth image
      imageWriter.SetFileName(data_dir+"/images/"+str(i).zfill(10)+"depth_ground_truth.png");
      imageWriter.SetInputConnection(scale.GetOutputPort());
      imageWriter.Write();
  
  renWin.Render();
  interactor.Start();