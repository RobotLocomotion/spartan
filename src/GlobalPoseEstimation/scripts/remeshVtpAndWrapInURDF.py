'''
Usage:

directorPython remeshVtpAndWrapInURDF.py <path/to/data.vtp> <target # of faces>

This script will read the vtp file and save a vtp file.
It'll resample the input VTP polyData based on the given
absolute uniform spacing, and if bounds are provided,
prune the point cloud to the given absolute bounding box.
'''

import os
from director import ioUtils
from director import segmentation
from director import vtkNumpy as vnp
import subprocess
import vtk

# Heavy reference to 
# https://gist.github.com/awesomebytes/a3bc8729d0c1d0a9499172b9a77d2622

# Script taken from doing the needed operation
# (Filters > Remeshing, Simplification and Reconstruction >
# Quadric Edge Collapse Decimation, with target # of ffaces
# 0.9 percentage reduction (10%), 0.3 Quality threshold (70%)
# Target number of faces is ignored with those parameters
# conserving face normals, planar simplification and
# post-simplimfication cleaning)
# And going to Filter > Show current filter script
filter_script_mlx = """<!DOCTYPE FilterScript>
<FilterScript>
 <filter name="Remove Duplicated Vertex">
 </filter>
 <filter name="Quadric Edge Collapse Decimation">
  <Param type="RichInt" value="%d" name="TargetFaceNum"/>
  <Param type="RichFloat" value="0" name="TargetPerc"/>
  <Param type="RichFloat" value="0.3" name="QualityThr"/>
  <Param type="RichBool" value="false" name="PreserveBoundary"/>
  <Param type="RichFloat" value="1" name="BoundaryWeight"/>
  <Param type="RichBool" value="false" name="PreserveNormal"/>
  <Param type="RichBool" value="false" name="PreserveTopology"/>
  <Param type="RichBool" value="true" name="OptimalPlacement"/>
  <Param type="RichBool" value="false" name="PlanarQuadric"/>
  <Param type="RichBool" value="false" name="QualityWeight"/>
  <Param type="RichBool" value="true" name="AutoClean"/>
  <Param type="RichBool" value="false" name="Selected"/>
 </filter>
</FilterScript>
"""

urdf_base = """
<?xml version="1.0"?>
<robot
  name="{name}">
  <link name="{name}">
    <visual>
      <geometry>
        <mesh filename="{visual_file}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="{collision_file}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
      <origin xyz="0 0 0"/>
    </inertial>
  </link>
</robot>
"""


def create_tmp_filter_file(faces, filename='filter_file_tmp.mlx'):
    with open('/tmp/' + filename, 'w') as f:
        f.write(filter_script_mlx % faces)
    return '/tmp/' + filename


if __name__ == '__main__':

    if len(sys.argv) != 3:
        print "Usage: directorPython remeshVtpAndWrapInURDF.py <path/to/data.vtp> <target # of faces>"
        exit(0)

    filename = sys.argv[1]
    num_faces = int(sys.argv[2])

    # Conversion pathway is vtp -> stl (using STLWriter internally)
    # stl -> obj in meshlab (so we have an obj to load in the URDF)
    # and obj -> vtp
    # STL is in the loop because VTK can't easily output objs
    filename_base, file_extension = os.path.splitext(filename)
    outfile_vtp_name = filename_base + "_" + str(num_faces) + "faces.vtp"
    outfile_obj_name = filename_base + "_" + str(num_faces) + "faces.obj"

    # CONVERSION TO STL
    polyData = ioUtils.readPolyData(filename)
    print 'input polydata has ', polyData.GetNumberOfPoints(), ' points, ', polyData.GetNumberOfPolys(), ' polys, and bounds ', polyData.GetBounds()
    ioUtils.writePolyData(polyData, filename_base + ".stl")

    # REMESHING WITH MESHLAB
    filter_script_path = create_tmp_filter_file(num_faces)
    command = "meshlabserver -i " + filename_base + ".stl"
    command += " -s " + filter_script_path
    command += " -o " + outfile_obj_name

    print "Going to execute: " + command
    output = subprocess.check_output(command, shell=True)
    last_line = output.splitlines()[-1]

    # CONVERSION BACK TO VTP
    polyData = ioUtils.readPolyData(outfile_obj_name)
    print 'output polydata has ', polyData.GetNumberOfPoints(), ' points, ', polyData.GetNumberOfPolys(), ' polys, and bounds ', polyData.GetBounds()
    ioUtils.writePolyData(polyData, outfile_vtp_name)

    # GENERATE A URDF
    with open(filename + "." + "decimated_" + str(num_faces) + ".urdf", 'w') as f:
        f.write(urdf_base.format(name = os.path.basename(filename_base), 
            visual_file = os.path.basename(outfile_obj_name), 
            collision_file = os.path.basename(outfile_obj_name)))