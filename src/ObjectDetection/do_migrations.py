#!/usr/bin/env python

import os, sys, signal
import yaml
import numpy as np
import matplotlib.pyplot as plt

''' Takes logs unsorted in base pose_est_pipeline directory and sorts them
to subfolders based on some hardcoded params. One-time utility... '''

ROOT_LOGS_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/CorlDev/data/logs_test/'

if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  subdirs = next(os.walk(ROOT_LOGS_DIR))[1]

  performance = {
    'goicp': dict(trans_error = [], angle_error = []),
    'fgr': dict(trans_error = [], angle_error = []),
    'super4pcs': dict(trans_error = [], angle_error = []),
    'mip': dict(trans_error = [], angle_error = [])
  }

  # Go collect the data
  for subdir in subdirs:
    complete_path = ROOT_LOGS_DIR + subdir

    paths = dict(
      mip = "%s/pose_est_pipeline/c%0.3f_s%0.3f_m%0.3f" % (complete_path, 0.25, 0.02, 0.02),
      goicp = "%s/pose_est_pipeline/c%0.3f_s%0.3f_m%0.3f" % (complete_path, 0.25, 0.005, 0.02),
      fgr = "%s/pose_est_pipeline/c%0.3f_s%0.3f_m%0.3f" % (complete_path, 0.25, 0.005, 0.005),
      super4pcs = "%s/pose_est_pipeline/c%0.3f_s%0.3f_m%0.3f" % (complete_path, 0.25, 0.005, 0.005) ,
    )
    
    for key in paths.keys():
      os.system("mkdir -p %s" % paths[key])
      os.system("mv %s_output.yaml %s" % (key, paths[key]))
      os.system("mv %s/pose_est_pipeline/%s_* %s" % (complete_path, key, paths[key]))





