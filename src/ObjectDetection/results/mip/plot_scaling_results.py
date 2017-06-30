#!/usr/bin/env python

import os, sys, signal
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import re
import math

if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  scene_scale_re = re.compile("o_[0-9][0-9]_s_[0-9][0-9]_ex[0-9][0-9]")

  subdirs = next(os.walk("."))[1]

  # Collect scenescale info
  scene_scale_results = dict(
    scenescales = [],
    runtimes = [],
    bounds = [])

  for subdir in sorted(subdirs):
    if scene_scale_re.match(subdir):
      output_file = "%s/scene_scale/cube_resampled/output.yaml" % subdir
      summary_file = "%s/scene_scale/cube_resampled/summary.yaml" % subdir
      config_file = "%s/config.yaml" % subdir

      outlier_num = int(subdir.split("_")[1])
      scene_num = int(subdir.split("_")[3])
      if (outlier_num != int(math.floor(scene_num / 3.))):
        continue
      

      if os.path.isfile(config_file) and os.path.isfile(output_file) and os.path.isfile(summary_file):
        output = yaml.load(open(output_file), yaml.CLoader)
        summary = yaml.load(open(summary_file), yaml.CLoader)
        config = yaml.load(open(config_file), yaml.CLoader)

        scene_scale_results["scenescales"].append(config["detector_options"]["downsample_to_this_many_points"])
        scene_scale_results["runtimes"].append(output["solutions"][0]["solve_time"])
        scene_scale_results["bounds"].append(output["solutions"][0]["bound"])

  # Collect model scale info
  model_scale_results = dict(
    scenescales = [],
    modelscales = [],
    runtimes = [],
    bounds = [])

  model_scale_subdir = re.compile("o_06_s_18_ex[0-9][0-9]")
  for subdir in sorted(subdirs):
    if model_scale_subdir.match(subdir):
      if not os.path.isdir(subdir + "/model_scale"):
        continue
      model_scale_examples = next(os.walk(subdir + "/model_scale"))[1]
      for example in model_scale_examples:
        model_scale = int(example.split("_")[2])

        output_file = "%s/model_scale/%s/output.yaml" % (subdir, example)
        summary_file = "%s/model_scale/%s/summary.yaml" % (subdir, example)
        config_file = "%s/config.yaml" % (subdir)

        if os.path.isfile(config_file) and os.path.isfile(output_file) and os.path.isfile(summary_file):
          output = yaml.load(open(output_file), yaml.CLoader)
          summary = yaml.load(open(summary_file), yaml.CLoader)
          config = yaml.load(open(config_file), yaml.CLoader)
          model_scale_results["scenescales"].append(config["detector_options"]["downsample_to_this_many_points"])
          model_scale_results["modelscales"].append(model_scale)
          model_scale_results["runtimes"].append(output["solutions"][0]["solve_time"])
          model_scale_results["bounds"].append(output["solutions"][0]["bound"])


  # Collect outlier scale info
  outlier_scale_results = dict(
    outlier_num = [],
    runtimes = [],
    bounds = [])

  outlier_scale_re = re.compile("o_[0-9][0-9]_s_18_ex[0-9][0-9]")
  for subdir in sorted(subdirs):
    if outlier_scale_re.match(subdir):
      output_file = "%s/scene_scale/cube_resampled/output.yaml" % subdir
      summary_file = "%s/scene_scale/cube_resampled/summary.yaml" % subdir
      config_file = "%s/config.yaml" % subdir

      outlier_num = int(subdir.split("_")[1])
      scene_num = int(subdir.split("_")[3])

      if os.path.isfile(config_file) and os.path.isfile(output_file) and os.path.isfile(summary_file):
        output = yaml.load(open(output_file), yaml.CLoader)
        summary = yaml.load(open(summary_file), yaml.CLoader)
        config = yaml.load(open(config_file), yaml.CLoader)

        outlier_scale_results["outlier_num"].append(outlier_num)
        outlier_scale_results["runtimes"].append(output["solutions"][0]["solve_time"])
        outlier_scale_results["bounds"].append(output["solutions"][0]["bound"])

  plt.figure()
  ax = plt.subplot(1, 3, 1)
  plt.scatter(scene_scale_results["scenescales"], scene_scale_results["runtimes"])
  for i in range(len(scene_scale_results["scenescales"])):
    runtime = scene_scale_results["runtimes"][i]
    scenescale = scene_scale_results["scenescales"][i]
    if runtime > 1200:
      ax.add_patch(
        patches.Rectangle(
          (runtime, scenescale),   # (x,y)
          1,          # width
          100,          # height
        )
      )
  plt.ylabel("Runtime (s)")
  plt.xlabel("# of scene pts")
  plt.ylim([0, 1200])
  plt.grid()

  plt.subplot(1, 3, 2)
  plt.scatter(model_scale_results["modelscales"], model_scale_results["runtimes"])
  plt.xlabel("# of model faces")
  plt.ylim([0, 1200])
  plt.grid()

  plt.subplot(1, 3, 3)
  plt.scatter(outlier_scale_results["outlier_num"], outlier_scale_results["runtimes"])
  plt.xlabel("# of outliers")
  plt.ylim([0, 1200])
  plt.grid()

  plt.show()


'''
  plt.figure()
  plt.subplot(2, 1, 1)
  plt.scatter(scene_scale_results["scenescales"], scene_scale_results["runtimes"])
  plt.ylabel("Runtime")
  plt.subplot(2, 1, 2)
  plt.ylabel("Bound")
  plt.scatter(scene_scale_results["scenescales"], scene_scale_results["bounds"])
  plt.xlabel("# of scene pts")


  plt.figure()
  plt.subplot(2, 1, 1)
  plt.scatter(model_scale_results["modelscales"], model_scale_results["runtimes"])
  plt.ylabel("Runtime")
  plt.subplot(2, 1, 2)
  plt.ylabel("Bound")
  plt.scatter(model_scale_results["modelscales"], model_scale_results["bounds"])
  plt.xlabel("# of model faces")
  plt.show()
'''