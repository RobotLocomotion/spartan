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
  '''
  // Modify the shader to color based on model normal
  // To do this we have to modify the vertex shader
  // to pass the normal in model coordinates
  // through to the fragment shader. By default the normal
  // is converted to View coordinates and then passed on.
  // We keep that, but add a varying for the original normal.
  // Then we modify the fragment shader to set the diffuse color
  // based on that normal. First lets modify the vertex
  // shader
  '''
  mapper.AddShaderReplacement(
    vtk.vtkShader.Vertex,
    "//VTK::Normal::Dec", #// replace the normal block
    True, #// before the standard replacements
    "//VTK::Normal::Dec\n" #// we still want the default
    "  varying vec3 myNormalMCVSOutput;\n", #//but we add this
    False #// only do it once
    );
  mapper.AddShaderReplacement(
    vtk.vtkShader.Vertex,
    "//VTK::Normal::Impl", #// replace the normal block
    True, #// before the standard replacements
    "//VTK::Normal::Impl\n" #// we still want the default
    "  myNormalMCVSOutput = normalMC;\n", #//but we add this
    False #// only do it once
    );
  #// now modify the fragment shader
  mapper.AddShaderReplacement(
    vtk.vtkShader.Fragment,  #// in the fragment shader
    "//VTK::Normal::Dec", #// replace the normal block
    True, #// before the standard replacements
    "//VTK::Normal::Dec\n" #// we still want the default
    "  varying vec3 myNormalMCVSOutput;\n", #//but we add this
    False #// only do it once
    );
  mapper.AddShaderReplacement(
    vtk.vtkShader.Fragment,  #// in the fragment shader
    "//VTK::Normal::Impl", #// replace the normal block
    True, #// before the standard replacements
    "//VTK::Normal::Impl\n" #// we still want the default calc
    "  diffuseColor = abs(myNormalMCVSOutput);\n", #//but we add this
    False #// only do it once
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

  actor = vtk.vtkActor();
  renderer = vtk.vtkRenderer();
  mapper = vtk.vtkOpenGLPolyDataMapper()
  renderer.SetBackground(0.0, 0.0, 0.0);
  renWin = vtk.vtkRenderWindow();
  renWin.SetSize(view_height,view_width)
  renWin.AddRenderer(renderer);
  renderer.AddActor(actor);
  iren = vtk.vtkRenderWindowInteractor()
  iren.SetRenderWindow(renWin);
  fileReader = vtk.vtkPLYReader()
  imageWriter = vtk.vtkPNGWriter()
  fileReader.SetFileName(sys.argv[1]+"/"+sys.argv[3])
  fileReader.Update();
  camera = vtk.vtkCamera()
  renderer.SetActiveCamera(camera);

  norms =vtk.vtkPolyDataNormals()
  norms.SetInputConnection(fileReader.GetOutputPort());
  norms.Update();
  mapper.SetInputConnection(norms.GetOutputPort());
  actor.SetMapper(mapper);

  #setup rendering enviornment
  windowToColorBuffer = vtk.vtkWindowToImageFilter()
  windowToColorBuffer.SetInput(renWin)
  windowToColorBuffer.SetInputBufferTypeToRGB()

  
  #setup camera calibration
  common.set_up_camera_params(camera)

  #shading
  set_material_prop(actor)
  set_shader_input(mapper)

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

      #write out depth image
      imageWriter.SetFileName(data_dir+"/images/"+str(i).zfill(10)+"normal_ground_truth.png");
      imageWriter.SetInputConnection(windowToColorBuffer.GetOutputPort());
      imageWriter.Write();
  
  renWin.Render();
  iren.Start();