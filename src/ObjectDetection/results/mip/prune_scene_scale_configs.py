import os
import yaml
import numpy as np
from math import floor

# Run with directorPython! No arguments needed.

if __name__ == '__main__':

    max_n_examples = 3

    for scene_scale in range(10, 45):

        for n_outliers in range(0, scene_scale):

            any_run_completed = False

            for n_ex in range(0, 4):
                target_dir = "o_%02d_s_%02d_ex%02d/" % (n_outliers, scene_scale, n_ex)

                if os.path.isdir(target_dir):
                  if os.path.isdir(target_dir + "/scene_scale/cube_resampled") and \
                     os.path.isfile(target_dir + "/scene_scale/cube_resampled/output.yaml") and \
                     os.path.isfile(target_dir + "/scene_scale/cube_resampled/summary.yaml"):
                     any_run_completed = True
                  elif scene_scale != 1000 and any_run_completed: # handles the outlier-scale exception case
                    print "Still waiting on target_dir %s" % target_dir