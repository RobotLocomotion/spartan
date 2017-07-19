import os
from director import ioUtils
from director import segmentation
from director import vtkNumpy as vnp
from director import transformUtils
from director import viewerclient
import vtk
import re
import yaml
import numpy as np

# Run with directorPython!

class MeshFile(viewerclient.BaseGeometry):
    __slots__ = ["filename", "scale"]
    def __init__(self, filename, scale=1.0):
        self.scale = scale
        self.filename = filename

    def serialize(self):
        return {
            "type": "mesh_file",
            "scale": [self.scale, self.scale, self.scale],
            "filename": self.filename
        }

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print "Usage: directorPython <script name> <class_regex> <instance_regex>"
        exit(1)

    class_regex = re.compile(sys.argv[1])
    instance_regex = re.compile(sys.argv[2])

    # We can provide an initial path if we want
    vis = viewerclient.Visualizer(path="/visualize_result/")

    # Start a thread to handle responses from the viewer. Doing this enables
    # the automatic reloading of missing geometry if the viewer is restarted.
    vis.start_handler()

    DATA_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/GlobalPoseEstimation/data/'
    CORL_ROOT_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/CorlDev/logs_test/'
    
    methods = subdirs = next(os.walk("./"))[1]

    for method in methods:
        configs = next(os.walk("%s/" % method))[1]
        for config in configs:
            classes = next(os.walk("%s/%s/" % (method, config)))[1]
            for class_name in classes:
                if not class_regex.match(class_name):
                    continue
                instances = next(os.walk("%s/%s/%s/" % (method, config, class_name)))[1]
                for instance in instances:
                    if not instance_regex.match(instance):
                        continue
                    this_data_dir = "%s/%s/%s/" % (DATA_DIR, class_name, instance)
                    if not os.path.isdir(this_data_dir):
                        continue

                    # OK, we have an example!
                    # Get the GT info from the original data
                    model_gt = yaml.load(open(this_data_dir + "ground_truth.yaml"))
                    model_name = DATA_DIR + "/" + model_gt["filename"]
                    q0 = model_gt["pose"][0]
                    [q0.append(x) for x in model_gt["pose"][1]]

                    # And the estimated pose from the method
                    model_est = yaml.load(open("%s/%s/%s/%s/output.yaml" % (method, config, class_name, instance)))
                    qest = model_est["solutions"][0]["models"][0]["q"]

                    if len(q0) == 6:
                        q0[3:7] = transformUtils.rollPitchYawToQuaternion(q0[3:6])
                    if len(qest) == 6:
                        qest[3:7] = transformUtils.rollPitchYawToQuaternion(qest[3:6])

                    gt_geom = viewerclient.GeometryData(MeshFile(model_name), color=[1, 0, 1, 0.5], transform=
                        transformUtils.getNumpyFromTransform(transformUtils.transformFromPose(
                            np.array(q0[0:3]), np.array(q0[3:7]))))
                    vis[class_name][instance]["gt"].setgeometry(gt_geom)

                    est_geom = viewerclient.GeometryData(MeshFile(model_name), color=[0, 1, 0, 0.5], transform=
                        transformUtils.getNumpyFromTransform((transformUtils.transformFromPose(
                            np.array(qest[0:3]), np.array(qest[3:7])))))

                    vis[class_name][instance][method].setgeometry(est_geom)

                    point_cloud_name = this_data_dir + "scene_cloud.vtp"
                    est_geom = viewerclient.GeometryData(MeshFile(point_cloud_name), color=[0, 0, 1, 0.5])
                    vis[class_name][instance]["scene_cloud"].setgeometry(est_geom)








