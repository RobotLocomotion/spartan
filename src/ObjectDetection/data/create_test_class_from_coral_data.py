#!/usr/bin/env python

import os, sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import re
import signal

from director import transformUtils

# Consumes a CorlDev log directory that contains a RECONSTRUCTED POINTCLOUD
# of a scene, and a REGISTRATION RESULT that describes where models are in the
# scene. Generates an test class from that data according to parameters set here.

CORL_DATA_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/CorlDev/data/logs_test/'
MODELS_ROOT_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/CorlDev/data/'


params = dict(
  scene_resample_spacing = 0.005,
  model_resample_spacing = 0.005,
  scene_crop_width = 0.25,
)

OUT_DATA_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/ObjectDetection/data/'
CLASS_DIR_NAME = "crop_%0.3f/" % (params["scene_crop_width"])

CLASS_NAME = "Crop %0.3f" % (params["scene_crop_width"])
CLASS_NOTES = ""

INSTANCE_PATTERN = re.compile(".*2017-06-13-40.*")


if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  # Make sure the data dir is a real directory
  if not os.path.isdir(CORL_DATA_DIR):
    print "Corl data directory ", data_dir, " doesn't exist."
    exit(0)

  scene_crop_width = params["scene_crop_width"]
  scene_resample_spacing = params["scene_resample_spacing"]
  model_resample_spacing = params["model_resample_spacing"]

  # Make sure the output directory exists
  os.system("mkdir -p " + OUT_DATA_DIR + "/" + CLASS_DIR_NAME)

  # Create a details.yaml file and spit it out
  detail_yaml = dict(
                   name = CLASS_NAME,
                   notes = CLASS_NOTES,
                   params = params
                   )

  with open(OUT_DATA_DIR + CLASS_DIR_NAME + "details.yaml", 'w') as f:
    yaml.dump(detail_yaml, f)


  # Iterate over data available in Corl
  subdirs = next(os.walk(CORL_DATA_DIR))[1]
  for subdir in subdirs:
    if not INSTANCE_PATTERN.match(subdir):
      continue
      
    complete_path = CORL_DATA_DIR + subdir

    # Sanity check a few things before we make any messes

    # Does the registration result exist?
    gt_pose_yaml_path = complete_path + "/registration_result.yaml"
    if not os.path.isfile(gt_pose_yaml_path):
      continue
    gt_pose_yaml = yaml.load(open(gt_pose_yaml_path))

    # Does the pointcloud exist?
    input_cloud = complete_path + "/reconstructed_pointcloud.vtp"
    if not os.path.isfile(input_cloud):
      continue
    
    # For each model...
    for model_name in gt_pose_yaml.keys():
      try:

        # Make sure we can find the model file
        model_path = MODELS_ROOT_DIR + '/' + gt_pose_yaml[model_name]["filename"]
        if not os.path.isfile(model_path):
          print "Registered model ", model_name, " has no model file available at ", model_path
          continue

        # Go on with the generation of this example
        this_output_dir = OUT_DATA_DIR + "/" + CLASS_DIR_NAME + "/" + subdir + "_" + model_name
        os.system("mkdir -p " + this_output_dir)

        # Copy over ground truth
        gt_yaml = gt_pose_yaml[model_name]
        with (open(this_output_dir + "/ground_truth.yaml", 'w')) as f:
          yaml.dump(gt_yaml, f)

        # Resample the point cloud as requested by our options -- cropping + point downsampling
        resampled_scene_file = "%s/scene_cloud.vtp" % (this_output_dir)
        scene_nx = float(gt_pose_yaml[model_name]["pose"][0][0]) - scene_crop_width / 2.
        scene_px = float(gt_pose_yaml[model_name]["pose"][0][0]) + scene_crop_width / 2.
        scene_ny = float(gt_pose_yaml[model_name]["pose"][0][1]) - scene_crop_width / 2.
        scene_py = float(gt_pose_yaml[model_name]["pose"][0][1]) + scene_crop_width / 2.
        scene_nz = float(gt_pose_yaml[model_name]["pose"][0][2]) - scene_crop_width / 2.
        scene_pz = float(gt_pose_yaml[model_name]["pose"][0][2]) + scene_crop_width / 2.
        
        command = "directorPython ../scripts/resampleVtp.py %s/reconstructed_pointcloud.vtp  %s %f %f %f %f %f %f %f" % (
            complete_path, resampled_scene_file, scene_resample_spacing, scene_nx, scene_px, scene_ny, scene_py, scene_nz, scene_pz
          )
        print "\n", command

        os.system(command)

        # Resample the object as requested by our options -- point downsampling
        resampled_model_file = "%s/model_cloud.vtp" % (this_output_dir)
        command = "directorPython ../scripts/resampleVtp.py %s %s %f" % (
            model_path, resampled_model_file, model_resample_spacing
          )
        print "\n", command
        os.system(command)

      except Exception as e:
        print "Unknown error while traversing Corl data: ", e
        print "Chugging onwards..."
        continue