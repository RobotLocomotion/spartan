#!/usr/bin/env python

import os, sys
import datetime
import signal
import re
import yaml
import argparse
from math import atan2

CLASS_PATTERN = re.compile("crop_model_0.020")
INSTANCE_PATTERN = re.compile("2017-06-.*-.*")
      
if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  num_features = 10000

  max_distance = 0.3

  bin_sizes = {
    "distance": 10,
    "n1_n2": 10,
    "d_n1": 10,
    "d_n2": 10
  }

  step_size = 0.2
  minimum_pts_in_cell = 100

  class_subdirs = next(os.walk("."))[1]
  for classname in class_subdirs:
    if CLASS_PATTERN.match(classname):
      instance_subdirs = next(os.walk("./%s" % classname))[1]
      for instancename in instance_subdirs:
        if INSTANCE_PATTERN.match(instancename):
          full_scene_path = "./%s/%s/scene_cloud_uncropped.vtp" % (classname, instancename)
          output_hist_file = "./%s/%s/ppf_histograms/" % (classname, instancename)
          
          # produce config file
          config_file = "./%s/%s/ppf_histogram_config.yaml" % (classname, instancename)
          config_params = dict(
            n_features = num_features,
            max_distance = max_distance,
            n_bins_distance = bin_sizes["distance"],
            n_bins_n1_n2 = bin_sizes["n1_n2"],
            n_bins_d_n1 = bin_sizes["d_n1"],
            n_bins_d_n2 = bin_sizes["d_n2"],
            step_size = step_size,
            minimum_pts_in_cell = minimum_pts_in_cell
          )
          with open(config_file, 'w') as f:
            yaml.dump(config_params, f)

          command = "compute_point_pair_feature_histogram %s %s %s" % (
            full_scene_path,
            output_hist_file,
            config_file
          )
          print "\n", command
          os.system(command)