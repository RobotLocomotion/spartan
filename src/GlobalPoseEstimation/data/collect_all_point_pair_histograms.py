#!/usr/bin/env python

import os
import re
import signal

CLASS_PATTERN = re.compile("crop_model_0.020")
INSTANCE_PATTERN = re.compile("2017-.*-.*-.*")
OUTPUT_FILE = "all_ppf_histogram.yaml"
PPFH_EXT = re.compile(".*\.ppfh")

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

  all_ppfs = []

  histograms = []

  class_subdirs = next(os.walk("."))[1]
  for classname in class_subdirs:
    if CLASS_PATTERN.match(classname):
      instance_subdirs = next(os.walk("./%s" % classname))[1]
      for instancename in instance_subdirs:
        if INSTANCE_PATTERN.match(instancename) and os.path.isdir("./%s/%s/ppf_histograms/" % (classname, instancename)):
          histogram_candidates = os.listdir("./%s/%s/ppf_histograms/" % (classname, instancename))
          histograms = histograms + ["./%s/%s/ppf_histograms/%s" % (classname, instancename, histogram) for histogram in histogram_candidates if PPFH_EXT.match(histogram)]

  # Use C++ utility to actually do the collection, as the C++ YAML parser
  # is much more efficient 
  print "Collecting data from ", len(histograms), " histograms..."

  with open("all_ppfs.txt", 'w') as f:
    f.write("\n".join(histograms))

  command = "collect_point_pair_feature_histograms all_ppfs.txt all_ppfs_summary.yaml"
  print command
  os.system(command)

