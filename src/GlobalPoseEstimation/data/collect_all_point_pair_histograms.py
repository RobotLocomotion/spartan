#!/usr/bin/env python

import os, sys
import datetime
import signal
import re
import argparse
import numpy as np
from math import atan2
import yaml

CLASS_PATTERN = re.compile("crop_model_0.020")
INSTANCE_PATTERN = re.compile(".*")
OUTPUT_FILE = "all_ppf_histogram.yaml"
      
if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  # TARGET PARAMETERS

  target_max_distance = 0.3
  dims = (10, 10, 10, 10)
  target_n_bins = {
    "distance": dims[0],
    "n1_n2": dims[1],
    "d_n1": dims[2],
    "d_n2": dims[3]
  }

  all_ppfs = np.zeros(dims)

  class_subdirs = next(os.walk("."))[1]
  for classname in class_subdirs:
    if CLASS_PATTERN.match(classname):
      instance_subdirs = next(os.walk("./%s" % classname))[1]
      for instancename in instance_subdirs:
        if INSTANCE_PATTERN.match(instancename):
          output_hist_file = "./%s/%s/scene_cloud_uncropped_ppf_histogram.yaml" % (classname, instancename)
          if os.path.isfile(output_hist_file):
            try:
              new_ppfs_yaml = yaml.load(open(output_hist_file))
              if new_ppfs_yaml["max_distance"] == target_max_distance and \
                  new_ppfs_yaml["n_bins"] == target_n_bins:
                 new_ppfs = np.reshape(new_ppfs_yaml["histogram"], dims, order="C")
                 all_ppfs += new_ppfs
                 print "Total points: ", np.sum(all_ppfs)
            except Exception as e:
              print "Exception ", e

  output = dict (
    max_distance = target_max_distance,
    n_bins = target_n_bins,
    histogram = [float(x) for x in list(np.reshape(all_ppfs, (dims[0]*dims[1]*dims[2]*dims[3])))]
  )

  with open(OUTPUT_FILE, 'w') as f:
    yaml.dump(output, f)
