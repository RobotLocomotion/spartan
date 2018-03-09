from director import mainwindowapp
from director import robotsystem
from director import applogic as app
from director import consoleapp
from director import transformUtils
from director import mainwindowapp
from director import depthscanner
from director import ioUtils
from director import filterUtils
from director import visualization as vis
import numpy as np
from director import objectmodel as om
from director import vtkAll as vtk
import Image
import yaml
import os
import subprocess


class Mesh(object):

    def __init__(self, path_to_bin="/home/drc/PoissonRecon/Bin/Linux",out_dir=".",out_file="meshed_scene.ply"):
        self.path_to_bin = path_to_bin
        self.out_file = out_file
        self.out_dir = out_dir

 # /media/drc/DATA/chris_labelfusion/logs_test/2017-06-16-15/original_log.lcmlog.ply
    def mesh_cloud(self, cloud_file,depth=10,pointWeight = 0):#catch errors and retur status
        print "no mesh found. Meshing point cloud scene."
        out = self.out_dir+"/"+self.out_file
        in_file = self.out_dir+"/"+cloud_file
        cmd = "{self.path_to_bin}/PoissonRecon --in {in_file}  \
        --out {out} --depth {depth} --pointWeight {pointWeight}".format(**locals())

        print cmd
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

        (o, e) = process.communicate()  
        status = process.wait()
        print "completed meshed point cloud"
        return 1

    def load_mesh(file_name):
        data = vtk.vtkPolyData()
        fileReader = vtk.vtkPLYReader()
        fileReader.SetFileName(file_name)
        data.SetInput(fileReader.GetOutputPort())
        return data



    