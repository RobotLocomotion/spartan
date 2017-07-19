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
    os.system("rm -r cube_resampled_*")

    # To generate the model cloud, resample a cube mesh
    base_mesh = "cube_excessive.obj"

    # Generate a base cube scene cloud that will be shared
    os.system("directorPython ../../scripts/resampleVtp.py %s base_model_cloud.vtp 0.02" % base_mesh)

    # Pick a transformation...
    ground_truth_config = dict(filename = base_mesh,
         pose = [[1.0, 0.0, 1.0], [-0.323, -0.602, 0.386, 0.620]])

    # Generate tf'd cube data for kicks
    polyData = ioUtils.readPolyData("base_model_cloud.vtp")
    
    tf = transformUtils.transformFromPose(ground_truth_config["pose"][0], ground_truth_config["pose"][1])
    tf_filter = vtk.vtkTransformPolyDataFilter()
    tf_filter.SetInput(polyData)
    tf_filter.SetTransform(tf)
    tf_filter.Update()
    tfd_data = tf_filter.GetOutput() 
    ioUtils.writePolyData(tfd_data, "base_scene_cloud.vtp")
    


    for model_scale in range(12, 32, 2):
        thisdir = "cube_resampled_%02d/" % model_scale
        os.system("mkdir %s" % thisdir)

        # copy in the cube mesh
        os.system("cp %s %s/cube.obj" % (base_mesh, thisdir))
        # and remesh it to have the desired # of faces
        os.system("directorPython ../../scripts/remeshVtpAndWrapInURDF.py %s/cube.obj %d" % (thisdir, model_scale))
        # copy urdf to the name expected by the run script... this is hacky
        os.system("cp %s/cube.obj.decimated_%02d.urdf %s/cube.obj.decimated_50.urdf" % (thisdir, model_scale, thisdir))

        # Write gt yaml
        ground_truth_config["filename"] = "%s/%s/cube.obj" % ("model_scale", thisdir)
        with open("%s/ground_truth.yaml" % thisdir, 'w') as f:
            yaml.dump(ground_truth_config, f)

        # copy in the scene and model clouds, which don't change
        os.system("cp base_model_cloud.vtp %s/model_cloud.vtp" % thisdir)
        os.system("cp base_scene_cloud.vtp %s/scene_cloud.vtp" % thisdir)







