import os,sys
sys.path.insert(0, '../')

import numpy as np
from scipy import misc
from common import common
from render import render_sim
from director import vtkAll as vtk
import yaml
import time
from director import vtkNumpy as vnp
from director import filterUtils
#from RGBDCNN import network

def set_shader(mapper):
	mapper.SetVertexShaderCode(
    "//VTK::System::Dec\n"  
    "attribute vec4 vertexMC;\n"
    "attribute vec3 normalMC;\n"
    "uniform mat3 normalMatrix;\n"
    "uniform mat4 MCDCMatrix;\n"
    "uniform mat4 MCVCMatrix;\n"
    "varying vec3 normalVCVSOutput;\n"
    "varying vec4 vertexVCVSOutput;\n"
    "attribute vec2 tcoordMC;\n"
    "varying vec2 tcoordVCVSOutput;\n"
    "void main () {\n"
    "  normalVCVSOutput = normalMatrix * normalMC;\n"
    "  tcoordVCVSOutput = tcoordMC;\n"
    "  vertexVCVSOutput = MCVCMatrix * vertexMC;\n"
    "  gl_Position = MCDCMatrix * vertexMC;\n"
    "}\n")

	mapper.SetFragmentShaderCode(
    "//VTK::System::Dec\n"  
    "//VTK::Output::Dec\n"  
    "varying vec3 normalVCVSOutput;\n"
    "varying vec4 vertexVCVSOutput;\n"
    "varying vec2 tcoordVCVSOutput;\n"
    "out vec4 colorOut;\n"
    "uniform float z_near;\n"
    "uniform float z_far;\n"
    "uniform sampler2D texture_0;\n"
    "void main () {\n"
    "  float z = -vertexVCVSOutput.z;  // In meters.\n"
    "  float random_noise = texture(texture_0, vec2(gl_FragCoord.x / 640., gl_FragCoord.y / 480.)).x;\n"
    # // "  float f = pow(dot(normalVCVSOutput, normalize(vertexVCVSOutput.xyz)), 5.) / pow(z, 2.);\n"
    # // "  float angle = dot(normalVCVSOutput, normalize(-vertexVCVSOutput.xyz)) / 2. + 0.5;\n"
    # // "  float distance = 1. / pow(z, 2.);\n"
    # // "  float dnoise = distance * (1. + 0.01 * random_noise);\n"
    # // "  float f = angle / pow(z, 2.) + random_noise;\n"
    # // "  float z = gl_FragCoord.z; // [0, 1].\n"
    "  float angle = dot(normalVCVSOutput, normalize(-vertexVCVSOutput.xyz));\n"
    "  if (angle + random_noise * 0.2 > 0.3 * min(1.0, z * 2.5 / 5.)) {\n"
    "    float z_noise = z * (1. + 0.01 * random_noise);\n"
    "    float z_norm = (z_noise - z_near) / (z_far - z_near); // From meters to [0, 1].\n"
    "    colorOut = vec4(z_norm, z_norm, z_norm, 1.);\n"
    "  } else {\n"
    "    colorOut = vec4(0, 0, 0, 1.);\n"
    "  }\n"
    "  float z_norm = (z - z_near) / (z_far - z_near); // From meters to [0, 1].\n"
    "  z_norm = clamp(z_norm, 0.f, 1.f);\n"
    "  colorOut = vec4(z_norm, z_norm, z_norm, 1.);\n"
    "}\n")


def vtkICP(model,scene):
		#set object prior loc here
		icp = vtk.vtkIterativeClosestPointTransform()
		icp.SetMaximumNumberOfIterations(10)
		#icp.StartByMatchingCentroidsOn()
		icp.SetSource(model)
		icp.SetTarget(scene)
		icp.GetLandmarkTransform().SetModeToRigidBody()
		icp.Modified()
		icp.Update()
		mat = icp.GetMatrix()
		t = vtk.vtkTransform()
		t.PostMultiply()
		t.SetMatrix(mat)
		return t
 
### enumerate tests try only cluttered scenes
objects_per_scene = 5
path  = "/home/drc/DATA/chris_labelfusion/CORL2017/logs_test/"
paths = []
for f in os.listdir(path):
	if "2017" in f:
		if "registration_result.yaml" in os.listdir(path+f):
			with open(path+f+"/registration_result.yaml") as read:
				transformYaml = yaml.load(read)
				if len(transformYaml.keys()) >=objects_per_scene:
					paths.append((f,transformYaml.keys())) 

print "extracting from scenes..."
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
#common.set_up_camera_params(camera1)
renSource = vtk.vtkRendererSource()
renSource.SetInput(renderer)
renSource.WholeWindowOff()
renSource.DepthValuesOnlyOn()
renSource.Update()

#####setup image filters
filter1= vtk.vtkWindowToImageFilter()
scale =vtk.vtkImageShiftScale()
filter1.SetInput(renWin)
filter1.SetMagnification(1)
filter1.SetInputBufferTypeToZBuffer()
windowToColorBuffer = vtk.vtkWindowToImageFilter()
windowToColorBuffer.SetInput(renWin)
windowToColorBuffer.SetInputBufferTypeToRGB()     
scale.SetOutputScalarTypeToUnsignedShort()
scale.SetScale(1000);

out_dir = "/home/drc/DATA/chris_labelfusion/RGBDCNN/"
object_dir = "/home/drc/DATA/chris_labelfusion/object-meshes"

out_file = "/home/drc/DATA/chris_labelfusion/RGBDCNN/stats.yaml"
stats = {}
samples_per_run = 1

###run through scenes
for i,j in paths[:1]:
  #set file names
  data_dir = path+i
  print data_dir
  data_dir_name =  os.path.basename(os.path.normpath(data_dir))
  object_dir = "/home/drc/DATA/chris_labelfusion/object-meshes"
  mesh ='meshed_scene.ply'

  #####set up mesh
  actor = vtk.vtkActor()
  mapper = vtk.vtkPolyDataMapper()
  #set_shader(mapper)
  fileReader = vtk.vtkPLYReader()
  fileReader.SetFileName(data_dir+"/"+mesh)
  mapper.SetInputConnection(fileReader.GetOutputPort())
  actor.SetMapper(mapper)
  renderer.AddActor(actor)

  #add objects
  objects = common.Objects(data_dir,object_dir)
  objects.loadObjectMeshes("/registration_result.yaml",renderer1,keyword=None)
  object_key = objects.objects.keys()[0]
  object_to_fit = objects.objects[object_key]
  ground_truth_pose = objects.poses[object_key]

  poses = common.CameraPoses(data_dir+"/posegraph.posegraph")
  for i in np.random.choice(range(1,500),samples_per_run):
  		#object_to_fit.VisibilityOn()

      # try:
		utimeFile = open(data_dir+"/images/"+ str(i).zfill(10) + "_utime.txt", 'r')
		utime = int(utimeFile.read())    

		#update camera transform
		cameraToCameraStart = poses.getCameraPoseAtUTime(utime)
		t = cameraToCameraStart
		common.setCameraTransform(camera, t)
		#common.setCameraTransform(camera1, t)
		renSource.Update()

		#get Depth image
		reader = vtk.vtkPNGReader()
		reader.SetFileName("/home/drc/DATA/chris_labelfusion/RGBDCNNTest/15predicted_depth.png")
		reader.Update()
		#img = reader.GetOutput().GetCellData().GetArray()
		#print np.shape(img)
		scale.SetInputConnection(reader.GetOutputPort())
		im_map = vtk.vtkDataSetMapper()
		im_map.SetInputConnection(scale.GetOutputPort())
		im_map.Update()
		#project depth into cloud
		pc = vtk.vtkDepthImageToPointCloud()

		#pc.SetInputConnection(0,renSource.GetOutputPort())
		pc.SetInputConnection(0,reader.GetOutputPort())

		pc.SetCamera(renderer1.GetActiveCamera())
		pc.CullNearPointsOn()
		#pc.CullFarPointsOn()
		pc.ProduceVertexCellArrayOff()
		pc.Update()
		pcMapper = vtk.vtkPointGaussianMapper()##???
		pcMapper.SetInputConnection(pc.GetOutputPort())
		pcMapper.EmissiveOff()
		pcMapper.SetScaleFactor(0.0)
		pcActor = vtk.vtkActor()
		pcActor.SetMapper(pcMapper)
		renderer1.AddActor(pcActor)
		imActor = vtk.vtkActor()
		imActor.SetMapper(im_map)
		#renderer1.AddActor(imActor)

		scene = pcActor.GetMapper().GetInput()
		#model = object_to_fit.GetMapper().GetInput()
		#object_to_fit.VisibilityOff()
		# icp = vtkICP(scene,model)
		# modelToSceneTransform = icp.GetLinearInverse()
		# alignedModel = filterUtils.transformPolyData(model, modelToSceneTransform)
		# print icp
		renWin.Render()

  		#renderer1.RemoveActor(pcActor);


  #renderer.RemoveAllViewProps();
  #renderer1.RemoveAllViewProps();

  renWin.Render();
renWin.Render();
interactor.Start();

