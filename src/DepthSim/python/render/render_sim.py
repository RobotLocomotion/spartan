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
import mesh_wrapper
import glob
import os

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

def render_depth(renWin,renderer,camera,data_dir,data_dir_name,num_im,out_dir,use_mesh,object_dir,mesh ='meshed_scene.ply',keyword=None):
  actor = vtk.vtkActor()
  filter1= vtk.vtkWindowToImageFilter()
  imageWriter = vtk.vtkPNGWriter()
  scale =vtk.vtkImageShiftScale()

  if use_mesh: #use meshed version of scene
    if not glob.glob(data_dir+"/"+mesh):
      out  = None
      if glob.glob(data_dir+"/original_log.lcmlog.ply"):
        out = "original_log.lcmlog.ply"
      elif glob.glob(data_dir+"/trimmed_log.lcmlog.ply"):
        out = "trimmed_log.lcmlog.ply"
      elif glob.glob('*.ply'):
        out = glob.glob('*.ply')[0]
      else:
         return
      mesher = mesh_wrapper.Mesh(out_dir = data_dir)
      status = mesher.mesh_cloud(out)
      print status
      #blocks until done
    mapper = vtk.vtkPolyDataMapper()
    fileReader = vtk.vtkPLYReader()
    fileReader.SetFileName(data_dir+"/"+mesh)
    mapper.SetInputConnection(fileReader.GetOutputPort())
    actor.SetMapper(mapper)
    renderer.AddActor(actor)
  else: #import just the objects
    objects = common.Objects(data_dir,object_dir)
    objects.loadObjectMeshes("/registration_result.yaml",renderer,keyword=keyword)

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
      try:
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
      except(IOError):
        break
  renderer.RemoveAllViewProps();
  renWin.Render();

def render_normals(renWin,renderer,camera,data_dir,data_dir_name,num_im,out_dir,use_mesh,object_dir,mesh ='meshed_scene.ply',keyword=None):
  #setup rendering enviornment
  actor = vtk.vtkActor()
  filter1= vtk.vtkWindowToImageFilter()
  imageWriter = vtk.vtkPNGWriter()
  scale =vtk.vtkImageShiftScale()

  if use_mesh: #use meshed version of scene
    if not glob.glob(data_dir+"/"+mesh):
      out  = "original_log.lcmlog.ply" if glob.glob(data_dir+"/original_log.lcmlog.ply") else "trimmed_log.lcmlog.ply"
      mesher = mesh_wrapper.Mesh(out_dir = data_dir)
      status = mesher.mesh_cloud(out)
      print status
      #blocks until done
    mapper = vtk.vtkPolyDataMapper()
    #shading
    #set_material_prop(actor)
    set_shader_input(mapper)
    fileReader = vtk.vtkPLYReader()
    fileReader.SetFileName(data_dir+"/"+mesh)
    mapper.SetInputConnection(fileReader.GetOutputPort())
    actor.SetMapper(mapper)
    renderer.AddActor(actor)

  else: #import just the objects
    objects = common.Objects(data_dir,object_dir)
    objects.loadObjectMeshes("/registration_result.yaml",renderer,keyword=keyword,shader = set_shader_input)


  #setup rendering enviornment
  windowToColorBuffer = vtk.vtkWindowToImageFilter()
  windowToColorBuffer.SetInput(renWin)
  windowToColorBuffer.SetInputBufferTypeToRGB()

  #setup camera calibration
  common.set_up_camera_params(camera)


  poses = common.CameraPoses(data_dir+"/posegraph.posegraph")
  for i in range(1,num_im+1):
    try:
      utimeFile = open(data_dir+"/images/"+ str(i).zfill(10) + "_utime.txt", 'r')
      utime = int(utimeFile.read())    

      #update camera transform
      cameraToCameraStart = poses.getCameraPoseAtUTime(utime)
      t = cameraToCameraStart
      common.setCameraTransform(camera, t)
      renWin.Render()

      windowToColorBuffer.Modified()
      windowToColorBuffer.Update()

      #write out depth image
      imageWriter.SetFileName(out_dir+str(i).zfill(10)+"_"+data_dir_name+"_normal_ground_truth.png");
      imageWriter.SetInputConnection(windowToColorBuffer.GetOutputPort());
      imageWriter.Write();
    except(IOError):
        break
  renderer.RemoveAllViewProps();
  renWin.Render();

if __name__ == '__main__':
  #setup
  view_height = 640
  view_width = 480
  data_dir = sys.argv[1]
  data_dir_name =  os.path.basename(os.path.normpath(data_dir))
  num_im = int(sys.argv[2])
  mesh = sys.argv[3]
  out_dir = sys.argv[1]+"/images/" if len(sys.argv)==3 else sys.argv[4]
  object_dir = "/home/drc/DATA/object-meshes"
  print "rendering Label Fusion data", data_dir_name
  use_mesh = False

  #setup rendering enviornment
  renderer = vtk.vtkRenderer()
  renWin = vtk.vtkRenderWindow()
  interactor = vtk.vtkRenderWindowInteractor()
  renWin.SetSize(view_height,view_width)
  camera = vtk.vtkCamera()
  renderer.SetActiveCamera(camera);
  renWin.AddRenderer(renderer);
  interactor.SetRenderWindow(renWin);
  common.set_up_camera_params(camera)


  render_normals(renWin,renderer,camera,data_dir,data_dir_name,num_im,mesh,out_dir+"normal/",use_mesh,object_dir)
  render_depth(renWin,renderer,camera,data_dir,data_dir_name,num_im,mesh,out_dir+"gtdepth/",use_mesh,object_dir)


  renWin.Render();
  interactor.Start();