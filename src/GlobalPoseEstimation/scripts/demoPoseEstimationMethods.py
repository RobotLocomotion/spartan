#!/usr/bin/env python
import os
from director import ioUtils
from director import segmentation
from director import vtkNumpy as vnp
from director import transformUtils
from director.thirdparty import transformations

import math
from math import atan2
import numpy as np
import vtk
import yaml
 
def get_difference_between_pose_and_output(target_pose, output_file):
  output_yaml = yaml.load(open(output_file))
  output_pose = output_yaml["solutions"][0]["models"][0]["q"]
  output_trans = np.array(output_pose[0:3])
  if len(output_pose) == 7:
    output_quat = np.array(output_pose[3:7])
  else:
    output_quat = transformUtils.rollPitchYawToQuaternion(np.array(output_pose[3:6]))
  target_trans = target_pose[0]
  target_quat = target_pose[1]

  trans_error = target_trans - output_trans

  d_1 = transformations.quaternion_multiply(output_quat, transformations.quaternion_inverse(target_quat))
  d_2 = transformations.quaternion_multiply(-output_quat, transformations.quaternion_inverse(target_quat))
  angle_error = min( 2*atan2(np.linalg.norm(d_1[1:]), d_1[0]), 2*atan2(np.linalg.norm(d_2[1:]), d_2[0]) )
  print "GT: ", target_trans, ", ", target_quat
  print "Estimated: ", output_trans, ", ", output_quat
  print "Trans error: ", list(trans_error)
  print "Angle error: ", angle_error

  return (trans_error, angle_error)


if __name__ == '__main__':
  test_data_dir = "/tmp/demoPoseEstimationMethods_workdir/"
  os.system("mkdir -p %s" % test_data_dir)

  # Generate a point cloud using the Drake point cloud generator
  pcg_output_file = test_data_dir + "model_cloud.vtp"
  pcg_gt_file = test_data_dir + "registration_result.yaml"
  scene_file = test_data_dir + "scene_cloud.vtp"

  pcg_config_file = os.getenv("SPARTAN_SOURCE_DIR") + \
    "/src/GlobalPoseEstimation/config/point_cloud_generator_config.yaml"
    
  os.system("run_point_cloud_generator %s %s" %
    (pcg_config_file, pcg_output_file))

  # Transform it to create a scene cloud
  pose = [[1.0, 1.0, 1.0], [0.5, 0.5, 0.5, 0.5]]
  pcg_config = yaml.load(open(pcg_config_file))

  # Pick a transformation...
  ground_truth_config = dict(filename =
         os.path.dirname(pcg_config_file) +
         pcg_config["point_cloud_generator_options"]["models"][0]["urdf"],
       pose = pose)
  # Write gt yaml
  with open(pcg_gt_file, 'w') as f:
      yaml.dump(ground_truth_config, f)

  # Generate tf'd cube data
  polyData = ioUtils.readPolyData(pcg_output_file)
  tf = transformUtils.transformFromPose(ground_truth_config["pose"][0], ground_truth_config["pose"][1])
  tf_filter = vtk.vtkTransformPolyDataFilter()
  tf_filter.SetInput(polyData)
  tf_filter.SetTransform(tf)
  tf_filter.Update()
  tfd_data = tf_filter.GetOutput() 
  ioUtils.writePolyData(tfd_data, scene_file)

  # Run pose estimation on it

  ### SUPER4PCS
  print "METHOD: Super4PCS\n********************************"
  base_config_file = os.getenv("SPARTAN_SOURCE_DIR") + \
        "/src/GlobalPoseEstimation/config/super4pcs_detector_ex.yaml"
  config = yaml.load(open(base_config_file))
  output_config_file = test_data_dir + "super4pcs_config.yaml"
  config["detector_options"]["overlap_estimation"] = 0.8
  config["detector_options"]["max_time_seconds"] = 10
  with open(output_config_file, 'w') as f:
    yaml.dump(config, f)
  est_output_file = test_data_dir + "super4pcs_output.yaml"
  os.system("run_super4pcs_pose_estimator %s %s %s %s" % (
        scene_file, pcg_output_file, output_config_file, est_output_file
    ))

  trans_error, angle_error = get_difference_between_pose_and_output(pose, est_output_file)
  if max(trans_error) > 0.1: # angle error is hard because of rotational symmetries
    exit(-1)




