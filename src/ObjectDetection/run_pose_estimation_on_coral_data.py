#!/usr/bin/env python

import os, sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import signal

# Consumes a CorlDev log directory that contains a RECONSTRUCTED POINTCLOUD
# of a scene, and a REGISTRATION RESULT that describes where models are in the
# scene. Invokes the requested requested pose estimation algorithm on the pointcloud,
# after performing a tunable amount of cropping and resampling.

# Make sure you have a drakevisualizer open when running this, so you can see
# the vis output.

DEFAULT_DATA_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/CorlDev/data/logs_test/2017-06-09-04/'
MODELS_ROOT_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/CorlDev/data/'
DETECTORS_CONFIG_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/ObjectDetection/config'
WORK_DIR_NAME = "pose_est_pipeline"

scene_resample_spacing = 0.02
scene_crop_width = 0.25

model_resample_spacing = 0.02

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

  # Make sure there's a sandbox directory within the data dir to spit our intermediate data into
  os.system("mkdir -p " + data_dir + "/" + WORK_DIR_NAME)


  # For each model in this cloud...
  gt_pose_yaml = yaml.load(open(data_dir + "/registration_result.yaml"))
  for model_name in gt_pose_yaml.keys():
    # Make sure we can find the model file
    model_path = MODELS_ROOT_DIR + '/' + gt_pose_yaml[model_name]["filename"]
    if not os.path.isfile(model_path):
      print "Registered model ", model_name, " has no model file available at ", model_path
      continue

    # Resample the point cloud as requested by our options -- cropping + point downsampling
    resampled_scene_file = "%s/%s/resampled_pointcloud.vtp" % (data_dir, WORK_DIR_NAME)
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
    resampled_model_file = "%s/%s/resampled_%s.vtp" % (data_dir, WORK_DIR_NAME, model_name)
    command = "directorPython scripts/resampleVtp.py %s %s %f" % (
        model_path, resampled_model_file, model_resample_spacing
      )
    print "\n", command
    os.system(command)

    # And finally invoke our pose est routine

    if detectorType == "mip":
      command = "run_miqp_mesh_model_detector %s %s/miqp_mesh_model_detector_ex.yaml %s/%s/miqp_mesh_model_output.yaml" % (
        resampled_scene_file, DETECTORS_CONFIG_DIR, data_dir, WORK_DIR_NAME
      )

    elif detectorType == "goicp":
      command = "run_goicp_detector %s %s %s/goicp_detector_ex.yaml %s/%s/goicp_output.yaml" % (
        resampled_scene_file, resampled_model_file, DETECTORS_CONFIG_DIR, data_dir, WORK_DIR_NAME
      )

    elif detectorType == "fgr":
      command = "run_fgr_detector %s %s %s/fgr_detector_ex.yaml %s/%s/fgr_output.yaml" % (
        resampled_scene_file, resampled_model_file, DETECTORS_CONFIG_DIR, data_dir, WORK_DIR_NAME
      )

    elif detectorType == "super4pcs":
      command = "run_super4pcs_detector %s %s %s/super4pcs_detector_ex.yaml %s/%s/super4pcs_output.yaml" % (
        resampled_scene_file, resampled_model_file, DETECTORS_CONFIG_DIR, data_dir, WORK_DIR_NAME
      )

    else:
      print "Detector type \"", detectorType, "\" not recognized."
      exit(0)

    print "\n", command
    os.system(command)
