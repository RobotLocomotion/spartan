#!/usr/bin/env python

import os, sys
import datetime
import signal
import re
import argparse
from math import atan2

CLASS_PATTERN = re.compile("crop_model_0.020")
INSTANCE_PATTERN = re.compile(".*")
      
if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  num_features = 100000

  max_distance = 0.3

  bin_sizes = {
    "distance": 10,
    "n1_n2": 10,
    "d_n1": 10,
    "d_n2": 10
  }

  class_subdirs = next(os.walk("."))[1]
  for classname in class_subdirs:
    if CLASS_PATTERN.match(classname):
      instance_subdirs = next(os.walk("./%s" % classname))[1]
      for instancename in instance_subdirs:
        if INSTANCE_PATTERN.match(instancename):
          full_scene_path = "./%s/%s/scene_cloud_uncropped.vtp" % (classname, instancename)
          output_hist_file = "./%s/%s/scene_cloud_uncropped_ppf_histogram.yaml" % (classname, instancename)
          command = "compute_point_pair_feature_histogram %s %s %d %d %f %d %d %d" % (
            full_scene_path,
            output_hist_file,
            num_features,
            bin_sizes["distance"],
            max_distance,
            bin_sizes["n1_n2"],
            bin_sizes["d_n1"],
            bin_sizes["d_n2"]
          )
          print "\n", command
          os.system(command)