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

  # Each type of solver has a dictionary that maps
  # parameter description strings (describing downsampling)
  # to dictionaries storing trans and angle error lists
  performance = {
    'goicp': dict(),
    'fgr': dict(),
    'super4pcs': dict(),
    'mip': dict()
  }

  # Go collect the data
  for subdir in subdirs:
    complete_path = ROOT_LOGS_DIR + subdir + "/pose_est_pipeline/"

    # All subdirs here are parameter sets for the point cloud downsampling
    possible_subdirs = next(os.walk(complete_path))[1]

    for param_dir in possible_subdirs:

      for key in performance.keys():
        target_summary_file = complete_path + "/%s/%s_summary.yaml" % (param_dir, key)
        if os.path.isfile(target_summary_file):
          if param_dir not in performance[key].keys():
            performance[key][param_dir] = dict(trans_error = [], angle_error = [])
          summary_yaml = yaml.load(open(target_summary_file))
          performance[key][param_dir]["angle_error"].append(summary_yaml["error"]["angle_error"])
          performance[key][param_dir]["trans_error"].append(np.linalg.norm(np.array(summary_yaml["error"]["trans_error"])))


  import matplotlib.cm as cm
  colors = ["green", "blue", "red"]


  # Plot performance scattergrams, color-coded by param type
  plt.figure()
  for i, key in enumerate(performance.keys()):
    plt.subplot(2, 2, i+1)
    #plt.hold(True)
    for j, param_string in enumerate(sorted(performance[key].keys())):
      plt.scatter(performance[key][param_string]["trans_error"], performance[key][param_string]["angle_error"], label=param_string, color=colors[j])

    plt.xlabel("Trans error")
    plt.ylabel("Angle error")
    plt.xlim([0, 1.0])
    plt.ylim([0, 3.14])
    plt.legend(loc='upper center', bbox_to_anchor=(0.8, 0.95))
    plt.title(key)
    plt.grid()


  # Plot performance histograms
  plt.figure()

  for i, key in enumerate(performance.keys()):
    plt.subplot(4, 2, 2*i+1)
    paramstrings = sorted(performance[key].keys())
    plt.hist([performance[key][param_string]["trans_error"] for param_string in paramstrings], bins=np.arange(0, 1.0, 0.1), label=paramstrings)

    plt.title(key)
    plt.ylabel("Trans error")
    plt.xlim([0, 1.0])
    plt.grid()

    plt.subplot(4, 2, 2*i+2)
    plt.hist([performance[key][param_string]["angle_error"] for param_string in paramstrings], bins=np.arange(0, 3.14, 0.1), label=paramstrings)
    plt.title(key)
    plt.ylabel("Angle error")
    plt.legend()
    plt.xlim([0, 3.14])
    plt.grid()
  

  # Plot performance as success ratio, per different scene cropping amounts
  trans_error_threshold = 0.05 # 5cm
  angle_error_threshold = 10 * 3.14 / 180 # 10 degrees
  ratios = {}
  unique_crop_amounts = []
  for key in performance.keys():
    ratios_by_crop_amount = {}
    for param_string in performance[key].keys():
      correct = 0.
      total = 0.

      scene_crop_amount = float(param_string.split("_")[0][1:-1])
      if scene_crop_amount not in unique_crop_amounts:
        unique_crop_amounts.append(scene_crop_amount)

      for trans_error, angle_error in zip(performance[key][param_string]["trans_error"], performance[key][param_string]["angle_error"]):
        if trans_error < trans_error_threshold and angle_error < angle_error_threshold:
          correct = correct + 1.
        total = total + 1.

      if scene_crop_amount in ratios_by_crop_amount.keys():
        ratios_by_crop_amount[scene_crop_amount][0] += correct
        ratios_by_crop_amount[scene_crop_amount][1] += total
        print "Warning, collapsing across params"
      else:
        ratios_by_crop_amount[scene_crop_amount] = [correct, total]

    ratios[key] = ratios_by_crop_amount

  plt.figure()
  plt.hold(True)
  eps = [np.linspace(0.1, 0.9, 2*len(unique_crop_amounts)+1)[x] for x in range(0, 2*len(unique_crop_amounts)+1, 2)]
  if (len(unique_crop_amounts) % 2 == 1):
    eps += [0.8 / float(len(unique_crop_amounts)) for x in eps]

  print eps
  for i, crop_amount in enumerate(sorted(unique_crop_amounts)):
    this_data = []
    for key in ratios.keys():
      if crop_amount in ratios[key].keys():
        this_data.append(ratios[key][crop_amount][0] / ratios[key][crop_amount][1])
      else:
        this_data.append(0.)
    plt.bar(np.arange(4) + eps[i], this_data, label="Crop %s" % crop_amount, color=colors[i], width = 0.8 / float(len(unique_crop_amounts)))

  plt.xticks(np.arange(0.5, 4.5, 1.0), [key for key in ratios.keys()])
  plt.xlim(0, 4)
  plt.ylabel("Percent Correct Classifications\n %0.2f trans tol and %0.3f radian angle tol" % (trans_error_threshold, angle_error_threshold))
  plt.grid()
  plt.legend()

  plt.show()







