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
from vtk.util.misc import vtkRegressionTestImage
import common as common

#TODO:one shading lib for both depth and normal
def set_shader_input(mapper):
  mapper.AddShaderReplacement(
    vtk.vtkShader.Fragment,  #// in the fragment shader
    "//VTK::Light::Impl", #// replace the normal block
    True, #// before the standard replacements
    "fragOutput0 = vec4((normalVCVSOutput.x+1)/2.0,(normalVCVSOutput.y+1)/2.0,(normalVCVSOutput.z+1)/2.0, 1);\n",
    True #// only do it once
    );

def set_material_prop(actor):
  actor.GetProperty().SetAmbientColor(0.2, 0.2, 1.0);
  actor.GetProperty().SetDiffuseColor(1.0, 0.65, 0.7);
  actor.GetProperty().SetSpecularColor(1.0, 1.0, 1.0);
  actor.GetProperty().SetSpecular(0.5);
  actor.GetProperty().SetDiffuse(0.7);
  actor.GetProperty().SetAmbient(0.5);
  actor.GetProperty().SetSpecularPower(20.0);
  actor.GetProperty().SetOpacity(1.0);

if __name__ == '__main__':
  #setup
  view_height = 640
  view_width = 480
  data_dir = sys.argv[1]
  num_im = int(sys.argv[2])
  mesh = sys.argv[3]
  use_mesh = True

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
    #shading
    #set_material_prop(actor)
    set_shader_input(mapper)
    fileReader = vtk.vtkPLYReader()
    fileReader.SetFileName(sys.argv[1]+"/"+sys.argv[3])
    mapper.SetInputConnection(fileReader.GetOutputPort())
    actor.SetMapper(mapper)
    renderer.AddActor(actor)
    norms =vtk.vtkPolyDataNormals()
    norms.SetInputConnection(fileReader.GetOutputPort());
    norms.Update();
    mapper.SetInputConnection(norms.GetOutputPort());
    actor.SetMapper(mapper);

  else: #import just the objects
    objects = common.Objects(data_dir,"/home/drc/spartan/Data_objects")
    objects.loadObjectMeshes("/registration_result.yaml",renderer,set_shader_input)


  #setup rendering enviornment
  windowToColorBuffer = vtk.vtkWindowToImageFilter()
  windowToColorBuffer.SetInput(renWin)
  windowToColorBuffer.SetInputBufferTypeToRGB()

  #setup camera calibration
  common.set_up_camera_params(camera)


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

      windowToColorBuffer.Modified()
      windowToColorBuffer.Update()

      #update norms
      #mapper.Update()

      #write out depth image
      imageWriter.SetFileName(data_dir+"/images/"+str(i).zfill(10)+"normal_ground_truth.png");
      imageWriter.SetInputConnection(windowToColorBuffer.GetOutputPort());
      imageWriter.Write();
  
  renWin.Render();
  interactor.Start();