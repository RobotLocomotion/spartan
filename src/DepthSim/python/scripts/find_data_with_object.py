import os,sys
sys.path.insert(0, '../')

import numpy as np
from scipy import misc
from common import common
import yaml
from render import render_sim
from director import vtkAll as vtk

#
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
	'''
	name = os.path.basename(os.path.normpath(path+i[0]))
	for j in os.listdir(path+i[0]+"/images/"):
		#os.system("cp "+ path+i+"/images/"+j "/home/drc/DATA/CORL2017/object_database/" j.split("_")[0]+"_"+name+"_normal_ground_truth.png")
		if "rgb" in j:
			os.system("cp " + path+i[0]+"/images/"+j +" /home/drc/DATA/CORL2017/object_real/" + j.split("_")[0]+"_"+name+"_rgb.png")
		elif "depth" in j:
			os.system("cp " +  path+i[0]+"/images/"+j +" /home/drc/DATA/CORL2017/object_real/" + j.split("_")[0]+"_"+name+"_depth.png")
'''

view_height = 480
view_width = 640
renderer = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
interactor = vtk.vtkRenderWindowInteractor()
renWin.SetSize(view_width,view_height)
camera = vtk.vtkCamera()
renderer.SetActiveCamera(camera);
renWin.AddRenderer(renderer);
interactor.SetRenderWindow(renWin);
common.set_up_camera_params(camera)

use_mesh = True
out_dir = "/home/drc/DATA/chris_labelfusion/RGBDCNN/"

mesh = "None"
for i,j in paths:

  data_dir = path+i
  data_dir_name =  os.path.basename(os.path.normpath(data_dir))
  num_im = 4000
  object_dir = "/home/drc/DATA/chris_labelfusion/object-meshes"

  if not os.path.exists(out_dir+data_dir_name):
    os.makedirs(out_dir+data_dir_name)

  print "rendering Label Fusion data", data_dir_name
  #render_sim.render_depth(renWin,renderer,camera,data_dir,data_dir_name,num_im,out_dir+data_dir_name+"/",use_mesh,object_dir)
  render_sim.render_normals(renWin,renderer,camera,data_dir,data_dir_name,num_im,out_dir+data_dir_name+"/",use_mesh,object_dir)
  #os.system("cp "+data_dir+"/images/*rgb.png "+ out_dir+data_dir_name)
  print "generated rgb images"
  #os.system("cp "+data_dir+"/images/*depth.png "+ out_dir+data_dir_name)
  print "generated real depth images"


renWin.Render();
interactor.Start();
