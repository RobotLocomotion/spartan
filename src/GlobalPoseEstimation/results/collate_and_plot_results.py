#!/usr/bin/env python

import os, sys, signal, re
import yaml
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  method_subdirs = next(os.walk("."))[1]

  # Each type of solver has a dictionary that maps
  # parameter description strings (describing downsampling)
  # to dictionaries storing trans and angle error lists
  performance = {
    'goicp': dict(),
    'super4pcs': dict(),
    'mip': dict()
  }

  n_classes = len(performance.keys())

  class_targets = ["crop_0.250", "crop_model_0.020"]
  instance_re = re.compile(".*_drill")

  # For every subfolder, which is a method...
  for method in method_subdirs:

    if method not in performance.keys():
      print "Skipping method ", method
      continue

    # For every configuration set...
    config_subdirs = next(os.walk("%s/" % method))[1]
    for config in config_subdirs:

      # For every instance class...
      class_subdirs = next(os.walk("%s/%s/" % (method, config)))[1]
      for class_name in class_subdirs:
        if class_name not in class_targets:
          print "Skipping class %s because it's not a target" % class_name
          continue
        if class_name in performance[method].keys():
          print "WARNING: class %s is repeated" % class_name
        performance[method][class_name] = dict(trans_error = [], angle_error = [], instance_names = [])

        # And for all instances in that class...
        instance_subdirs = next(os.walk("%s/%s/%s/" % (method, config, class_name)))[1]
        for instance in instance_subdirs:
          if not instance_re.match(instance):
            continue

          complete_path = "%s/%s/%s/%s/" % (method, config, class_name, instance)

          target_summary_file = complete_path + "summary.yaml"
          if os.path.isfile(target_summary_file):
            summary_yaml = yaml.load(open(target_summary_file))
            performance[method][class_name]["angle_error"].append(summary_yaml["error"]["angle_error"])
            performance[method][class_name]["trans_error"].append(np.linalg.norm(np.array(summary_yaml["error"]["trans_error"])))
            performance[method][class_name]["instance_names"].append(instance)

`
  import matplotlib.cm as cm
  colors = ["green", "blue", "red"]

  # Plot performance scattergrams, color-coded by param type
  plt.figure()
  for i, method in enumerate(performance.keys()):
    plt.subplot(2, 2, i+1)
    #plt.hold(True)
    for j, class_name in enumerate(sorted(performance[method].keys())):
      plt.scatter(performance[method][class_name]["trans_error"], performance[method][class_name]["angle_error"], label=class_name, color=colors[j])

    plt.xlabel("Trans error")
    plt.ylabel("Angle error")
    plt.xlim([0, 1.0])
    plt.ylim([0, 3.14])
    plt.legend(loc='upper center', bbox_to_anchor=(0.8, 0.95))
    plt.title(method)
    plt.grid()


  # Plot performance histograms
  plt.figure()

  for i, method in enumerate(performance.keys()):
    plt.subplot(n_classes, 2, 2*i+1)
    paramstrings = sorted(performance[method].keys())

    if len(paramstrings) > 0:
      plt.hist([performance[method][class_name]["trans_error"] for class_name in paramstrings], bins=np.arange(0, 1.0, 0.1), label=paramstrings)
      plt.ylim([0, 250])
      plt.ylabel(method + "\n# of Examples")
      if (i == n_classes-1):
        plt.xlabel("Trans. error")

      plt.xlim([0, 1.0])
      plt.grid()

      plt.subplot(n_classes, 2, 2*i+2)
      plt.hist([performance[method][class_name]["angle_error"] for class_name in paramstrings], bins=np.arange(0, 3.14, 0.1), label=paramstrings)
      if (i == n_classes-1):
        plt.xlabel("Angle error")
      plt.legend()
      plt.xlim([0, 3.14])
      plt.ylim(0, 50)
      plt.grid()
  

  # Plot performance as success ratio, per each class
  trans_error_threshold = 0.05 # 5cm
  angle_error_threshold_deg = 10
  angle_error_threshold = angle_error_threshold_deg * 3.14 / 180 # 10 degrees
  ratios = {}
  unique_classes = []

  correctness_by_method_and_example_index = {}
  example_names_in_order = []

  for method in performance.keys():
    correctness_by_method_and_example_index[method] = []

    ratios_by_class_name = {}
    for class_name in sorted(performance[method].keys()):
      correct = 0.
      total = 0.

      if class_name not in unique_classes:
        unique_classes.append(class_name)

      for trans_error, angle_error, instance_name in \
                                      zip(performance[method][class_name]["trans_error"], 
                                          performance[method][class_name]["angle_error"],
                                          performance[method][class_name]["instance_names"]):
        if trans_error < trans_error_threshold and angle_error < angle_error_threshold:
          correct = correct + 1.
          correctness_by_method_and_example_index[method].append(1)
        else:
          correctness_by_method_and_example_index[method].append(0)
        total = total + 1.
        example_names_in_order.append(class_name + "/" + instance_name)

      if class_name in ratios_by_class_name.keys():
        ratios_by_class_name[class_name][0] += correct
        ratios_by_class_name[class_name][1] += total
        print "Warning, collapsing across params"
      else:
        ratios_by_class_name[class_name] = [correct, total]

    ratios[method] = ratios_by_class_name

  print ratios
  for i in range(len(correctness_by_method_and_example_index[performance.keys()[0]])):
    #if correctness_by_method_and_example_index["mip"][i]:
      print "Example %s: %s" % (
          example_names_in_order[i], 
          ", ".join(["%s: %d" % (key, correctness_by_method_and_example_index[key][i]) for key in performance.keys()])
        )

  plt.figure()
  plt.hold(True)
  bar_width = 1.0 / float(len(unique_classes))
  eps = np.linspace(0.0, 1.0 - bar_width, len(unique_classes))

  print eps
  x_tick = []
  label = []
  print "%d unique classes, so bar width %f" % (len(unique_classes), bar_width)

  for i, class_name in enumerate(sorted(unique_classes)):
    this_data = []
    lens = []
    for method in sorted(ratios.keys()):
      if class_name in ratios[method].keys():
        this_data.append(100.*ratios[method][class_name][0] / ratios[method][class_name][1])
        lens.append(ratios[method][class_name][1])
      else:
        this_data.append(0.)
        lens.append(0)
    plt.bar(np.linspace(eps[i], eps[i] + n_classes, n_classes), this_data, label="Class %s" % class_name, color=colors[i], width = bar_width)
    [x_tick.append(x) for x in np.linspace(eps[i] + bar_width / 2., n_classes + eps[i] + bar_width / 2., n_classes)]
    [label.append("%s\nN=%d" % (key, N)) for (key, N) in zip(sorted(ratios.keys()), lens)]

  plt.xticks(x_tick, label)

  plt.ylim(0, 100.)
  plt.xlim(0, n_classes+1)
  plt.ylabel("Percent Correct Classifications") # % (trans_error_threshold, angle_error_threshold_deg))
  plt.grid()
  plt.legend()

  plt.show()







