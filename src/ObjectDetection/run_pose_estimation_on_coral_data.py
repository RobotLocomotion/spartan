#!/usr/bin/env python

import os, sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import signal

from director import transformUtils

# Consumes a CorlDev log directory that contains a RECONSTRUCTED POINTCLOUD
# of a scene, and a REGISTRATION RESULT that describes where models are in the
# scene. Invokes the requested requested pose estimation algorithm on the pointcloud,
# after performing a tunable amount of cropping and resampling.

# Make sure you have a drakevisualizer open when running this, so you can see
# the vis output.

DEFAULT_DATA_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/CorlDev/data/logs_test/2017-06-09-04/'
MODELS_ROOT_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/CorlDev/data/'
DETECTORS_CONFIG_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/ObjectDetection/config'

params = dict(
  mip = dict(
    scene_resample_spacing = 0.02,
    scene_crop_width = 0.3,
    model_resample_spacing = 0.02
  ),
  goicp = dict(
    scene_resample_spacing = 0.005,
    scene_crop_width = 0.3,
    model_resample_spacing = 0.02
  ),
  super4pcs = dict(
    scene_resample_spacing = 0.005,
    scene_crop_width = 0.3,
    model_resample_spacing = 0.005
  ),
  fgr = dict(
    scene_resample_spacing = 0.005,
    scene_crop_width = 0.3,
    model_resample_spacing = 0.005
  )
)

WORK_DIR_NAME_BASE = "pose_est_pipeline/c%0.3f_s%0.3f_m%0.3f/"


if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  if len(sys.argv) != 3:
    print "Usage:"
    print "\t<script name> <detector type> <CorlDev log dir>"
    exit(0)

  detectorType = sys.argv[1]
  detectorTypes = ["mip", "super4pcs", "goicp", "fgr"]
  if detectorType not in detectorTypes:
    print "Supplied detector type \"", detectorType, "\" must be one of [", " ".join(detectorTypes), "]"
    exit(0)

  data_dir = sys.argv[2]
  # Make sure the data dir is a real directory
  if not os.path.isdir(data_dir):
    print "Data directory ", data_dir, " doesn't exist."
    exit(0)
  # TODO(gizatt) other sanity-checks for existing files?

  scene_crop_width = params[detectorType]["scene_crop_width"]
  scene_resample_spacing = params[detectorType]["scene_resample_spacing"]
  model_resample_spacing = params[detectorType]["model_resample_spacing"]

  work_dir_name = WORK_DIR_NAME_BASE % (
      scene_crop_width, scene_resample_spacing, model_resample_spacing 
    )
  # Make sure there's a sandbox directory within the data dir to spit our intermediate data into
  os.system("mkdir -p " + data_dir + "/" + work_dir_name)


  # For each model in this cloud...
  gt_pose_yaml = yaml.load(open(data_dir + "/registration_result.yaml"))
  for model_name in gt_pose_yaml.keys():
    # Make sure we can find the model file
    model_path = MODELS_ROOT_DIR + '/' + gt_pose_yaml[model_name]["filename"]
    if not os.path.isfile(model_path):
      print "Registered model ", model_name, " has no model file available at ", model_path
      continue

    # Resample the point cloud as requested by our options -- cropping + point downsampling
    resampled_scene_file = "%s/%s/resampled_pointcloud.vtp" % (data_dir, work_dir_name)
    scene_nx = float(gt_pose_yaml[model_name]["pose"][0][0]) - scene_crop_width / 2.
    scene_px = float(gt_pose_yaml[model_name]["pose"][0][0]) + scene_crop_width / 2.
    scene_ny = float(gt_pose_yaml[model_name]["pose"][0][1]) - scene_crop_width / 2.
    scene_py = float(gt_pose_yaml[model_name]["pose"][0][1]) + scene_crop_width / 2.
    scene_nz = float(gt_pose_yaml[model_name]["pose"][0][2]) - scene_crop_width / 2.
    scene_pz = float(gt_pose_yaml[model_name]["pose"][0][2]) + scene_crop_width / 2.
    command = "directorPython scripts/resampleVtp.py %s/above_table_pointcloud.vtp  %s %f %f %f %f %f %f %f" % (
        data_dir, resampled_scene_file, scene_resample_spacing, scene_nx, scene_px, scene_ny, scene_py, scene_nz, scene_pz
       )
    command = "directorPython scripts/resampleVtp.py %s/reconstructed_pointcloud.vtp  %s %f %f %f %f %f %f %f" % (
        data_dir, resampled_scene_file, scene_resample_spacing, scene_nx, scene_px, scene_ny, scene_py, scene_nz, scene_pz
      )
    print "\n", command
    os.system(command)

    # Resample the object as requested by our options -- point downsampling
    resampled_model_file = "%s/%s/resampled_%s.vtp" % (data_dir, work_dir_name, model_name)
    command = "directorPython scripts/resampleVtp.py %s %s %f" % (
        model_path, resampled_model_file, model_resample_spacing
      )
    print "\n", command
    os.system(command)

    # And finally invoke our pose est routine
    output_file = "%s/%s/%s_output.yaml" % (data_dir, work_dir_name, detectorType)
    if detectorType == "mip":
      command = "run_miqp_mesh_model_detector %s %s/miqp_mesh_model_detector_ex.yaml %s" % (
        resampled_scene_file, DETECTORS_CONFIG_DIR, output_file
      )
    elif detectorType == "goicp":
      command = "run_goicp_detector %s %s %s/goicp_detector_ex.yaml %s" % (
        resampled_scene_file, resampled_model_file, DETECTORS_CONFIG_DIR, output_file
      )
    elif detectorType == "fgr":
      command = "run_fgr_detector %s %s %s/fgr_detector_ex.yaml %s" % (
        resampled_scene_file, resampled_model_file, DETECTORS_CONFIG_DIR, output_file
      )
    elif detectorType == "super4pcs":
      command = "run_super4pcs_detector %s %s %s/super4pcs_detector_ex.yaml %s" % (
        resampled_scene_file, resampled_model_file, DETECTORS_CONFIG_DIR, output_file
      )
    else:
      print "Detector type \"", detectorType, "\" not recognized."
      exit(0)
    
    print "\n", command
    os.system(command)

    # Load up the output file and produce a score based on the accuracy of the TF
    output_yaml = yaml.load(open(output_file))
    output_pose = output_yaml["solutions"][0]["models"][0]["q"]
    output_trans = np.array(output_pose[0:3])
    if len(output_pose) == 7:
      output_quat = np.array(output_pose[3:7])
    else:
      output_quat = transformUtils.rollPitchYawToQuaternion(np.array(output_pose[3:6]))

    target_trans = np.array(gt_pose_yaml[model_name]["pose"][0])
    target_quat = np.array(gt_pose_yaml[model_name]["pose"][1])

    trans_error = target_trans - output_trans
    angle_error = np.arccos( (np.dot(output_quat, target_quat)) / (np.linalg.norm(target_quat) * np.linalg.norm(output_quat)))
    print "GT: ", target_trans, ", ", target_quat
    print "Estimated: ", output_trans, ", ", output_quat
    print "Trans error: ", list(trans_error)
    print "Angle error: ", angle_error

    # Collect those results into an appropriately named results file, along with the downsampling params used, and 
    # save it out.
    final_results_file = "%s/%s/%s_summary.yaml" % (data_dir, work_dir_name, detectorType)
    out_results = dict(
                   estimated = dict(
                    est_trans = output_trans.tolist(),
                    est_quat = output_quat.tolist()),
                   target = dict(
                    target_trans = target_trans.tolist(),
                    target_quat = target_quat.tolist()),
                   error = dict(
                    trans_error = trans_error.tolist(),
                    angle_error = float(angle_error)),
                   params = dict(
                    scene_resample_spacing = scene_resample_spacing,
                    scene_crop_width = scene_crop_width,
                    model_resample_spacing = model_resample_spacing),
                   detectorType = detectorType)

    with open(final_results_file, 'w') as f:
      yaml.dump(out_results, f)