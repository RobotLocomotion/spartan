#!/usr/bin/env python

import os, sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import signal
import re
import hashlib
import argparse
from math import atan2

from director import transformUtils
from director.thirdparty import transformations

# Make sure you have a drakevisualizer open when running this, so you can see
# the vis output.

DATA_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/GlobalPoseEstimation/data/'

TARGET_CLASSES = ["crop_0.250"] # leave empty for all targets
INSTANCE_PATTERN = re.compile(".*")


def do_update(method, config_dir, rebuild_all):
  ''' Given a configuration file, iterates through the data
  directory, checks if each example instance has been run already
  and needs an update, and does the updates if necessary for each
  method that we know how to handle. '''


        

if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  parser = argparse.ArgumentParser(description='Mess around with the model files.')
  parser.add_argument("-g", help="Generate 50-subdivision versions.", action='store_true')

  args = parser.parse_args(sys.argv[1:])

  # Iterate through available data...
  if not os.path.isdir(DATA_DIR):
    print "Data directory ", data_dir, " doesn't exist."
    exit(0)

  model_file_registry = {}

  class_names = next(os.walk(DATA_DIR))[1]
  for class_name in sorted(class_names):
    if class_name not in TARGET_CLASSES:
      continue

    # For each test instance...
    test_instances = next(os.walk(DATA_DIR + "/" + class_name))[1]
    for test_instance in sorted(test_instances):
      if not INSTANCE_PATTERN.match(test_instance):
        continue

      input_dir = "%s/%s/%s/" % (DATA_DIR, class_name, test_instance)
      # Investigate what model file this points to
      model_gt = yaml.load(open(input_dir + "/ground_truth.yaml"))
      model_file_base = DATA_DIR + "/" + model_gt["filename"]
      model_file_decimated_50 = DATA_DIR + "/" + model_gt["filename"] + ".decimated_50.urdf"

      model_file_exists = os.path.isfile(model_file_base)
      decimated_50_exists = os.path.isfile(model_file_decimated_50)

      if model_file_base not in model_file_registry.keys():
        model_file_registry[model_file_base] = dict(model_file_exists = model_file_exists,
                              decimated_50_exists = decimated_50_exists,
                              references = 1,
                              base_file = model_file_base)
      else:
        model_file_registry[model_file_base]["references"] += 1

  for key in model_file_registry.keys():
    print "Exists: %1d, Dec?: %1d, Refs: %05d, Name: %s" % (model_file_registry[key]["model_file_exists"], 
      model_file_registry[key]["decimated_50_exists"], model_file_registry[key]["references"], key)

    if model_file_registry[key]["model_file_exists"] == 1 and model_file_registry[key]["decimated_50_exists"] == 0 and args.g:
      command = "directorPython %s/../scripts/remeshVtpAndWrapInURDF.py %s 50" % (
        DATA_DIR, model_file_registry[key]["base_file"]
      )
      print "\n", command
      os.system(command)
