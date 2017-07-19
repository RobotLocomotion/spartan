import os
from director import ioUtils
from director import segmentation
from director import vtkNumpy as vnp
from director import transformUtils
import vtk
import yaml

# Run with directorPython! No arguments needed.

if __name__ == '__main__':

    # Clean out old data
    os.system("rm -r cube_resampled")
    os.system("mkdir cube_resampled")

    # To generate the model cloud, resample a cube mesh
    base_mesh = "object-meshes/cube.obj"

    os.system("directorPython ../../scripts/resampleVtp.py ../%s cube_resampled/model_cloud.vtp 0.02" % base_mesh)

    # Pick a transformation...
    ground_truth_config = dict(filename = base_mesh,
         pose = [[1.0, 0.0, 1.0], [-0.323, -0.602, 0.386, 0.620]])

    # Write gt yaml
    with open("cube_resampled/ground_truth.yaml", 'w') as f:
        yaml.dump(ground_truth_config, f)

    # Generate tf'd cube data
    polyData = ioUtils.readPolyData("cube_resampled/model_cloud.vtp")
    
    tf = transformUtils.transformFromPose(ground_truth_config["pose"][0], ground_truth_config["pose"][1])
    tf_filter = vtk.vtkTransformPolyDataFilter()
    tf_filter.SetInput(polyData)
    tf_filter.SetTransform(tf)
    tf_filter.Update()
    tfd_data = tf_filter.GetOutput() 
    ioUtils.writePolyData(tfd_data, "cube_resampled/scene_cloud.vtp")
    



