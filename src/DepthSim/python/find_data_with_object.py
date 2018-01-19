import os 
import numpy as np
from scipy import misc
from common import common
import yaml
from render import render_sim
from director import vtkAll as vtk

path  = "/home/drc/spartan/Data_ALL/logs_test/"
paths = []
for f in os.listdir(path):
	if "2017" in f:
		if "registration_result.yaml" in os.listdir(path+f):
			with open(path+f+"/registration_result.yaml") as read:
				transformYaml = yaml.load(read)
				if len(transformYaml.keys())==1 and transformYaml.keys()[0]=="drill":
					paths.append(f) 
print paths

view_height = 640
view_width = 480
renderer = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
interactor = vtk.vtkRenderWindowInteractor()
renWin.SetSize(view_height,view_width)
camera = vtk.vtkCamera()
renderer.SetActiveCamera(camera);
renWin.AddRenderer(renderer);
interactor.SetRenderWindow(renWin);
common.set_up_camera_params(camera)

use_mesh = False
out_dir = "/home/drc/spartan/Data_ALL/gan_data/sim/train/"

mesh = "None"
for i in paths:

  data_dir = path+i
  data_dir_name =  os.path.basename(os.path.normpath(data_dir))
  num_im = 4000


  print "rendering Label Fusion data", data_dir_name
  render_sim.render_depth(renWin,renderer,camera,data_dir,data_dir_name,num_im,mesh,out_dir,use_mesh,view_height,view_width)


renWin.Render();
interactor.Start();
