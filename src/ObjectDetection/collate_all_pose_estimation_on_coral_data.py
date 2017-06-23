#!/usr/bin/env python

import os, sys, signal
import yaml
import numpy as np
import matplotlib.pyplot as plt

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

    for key in performance.keys():
      target_summary_file = complete_path + "/pose_est_pipeline/%s_summary.yaml" % key
      if os.path.isfile(target_summary_file):
        summary_yaml = yaml.load(open(target_summary_file))
        performance[key]["angle_error"].append(summary_yaml["error"]["angle_error"])
        performance[key]["trans_error"].append(np.linalg.norm(np.array(summary_yaml["error"]["trans_error"])))

  # Plot performance scattergrams
  plt.figure()
  for i, key in enumerate(performance.keys()):
    plt.subplot(2, 2, i+1)
    plt.scatter(performance[key]["trans_error"], performance[key]["angle_error"])
    plt.xlabel("Trans error")
    plt.ylabel("Angle error")
    plt.title(key)
    plt.grid()


  # Plot performance histograms
  plt.figure()

  for i, key in enumerate(performance.keys()):
    plt.subplot(4, 2, 2*i+1)
    plt.hist(performance[key]["trans_error"], bins=np.arange(0, 1.0, 0.1))

    plt.title(key)
    plt.ylabel("Trans error")
    plt.xlim([0, 1.0])
    plt.grid()

    plt.subplot(4, 2, 2*i+2)
    plt.hist(performance[key]["angle_error"], bins=np.arange(0, 3.14, 0.1))
    plt.title(key)
    plt.ylabel("Angle error")
    plt.xlim([0, 3.14])
    plt.grid()
  
  # Plot performance as success ratio
  trans_error_threshold = 0.05 # 5cm
  angle_error_threshold = 10 * 3.14 / 180 # 10 degrees
  ratios = {}
  for key in performance.keys():
    correct = 0.
    total = 0.
    for trans_error, angle_error in zip(performance[key]["trans_error"], performance[key]["angle_error"]):
      if trans_error < trans_error_threshold and angle_error < angle_error_threshold:
        correct = correct + 1.
      total = total + 1.

    ratios[key] = correct / total

  plt.figure()
  plt.bar(np.arange(4), [ratios[key] for key in ratios], 1)
  plt.xticks(np.arange(0.5, 4.5, 1.0), [key + ", N=%d" % (len(performance[key]["trans_error"])) for key in ratios.keys()])
  plt.ylabel("Percent Correct Classifications\n %0.2f trans tol and %0.1f degree angle tol" % (trans_error_threshold, angle_error_threshold * 180 / 3.14))
  plt.grid()

  plt.show()







