import os
import yaml
import numpy as np
from math import ceil

# Run with directorPython! No arguments needed.

if __name__ == '__main__':

    # load up default scene scale config
    base_config = yaml.load(open("scene_scale_base_config.yaml"))

    n_examples = 3

    for scene_scale in range(10, 45):
        base_config["detector_options"]["downsample_to_this_many_points"] = scene_scale
        base_config["detector_options"]["add_this_many_outliers"] = int(ceil(scene_scale / 3))
        for i in range(n_examples):
            new_dir = "o_%02d_s_%02d_ex%02d/" % (base_config["detector_options"]["add_this_many_outliers"], base_config["detector_options"]["downsample_to_this_many_points"], i)
            os.system("mkdir %s -p" % new_dir)
            with open("%s/config.yaml" % new_dir, 'w') as f:
                yaml.dump(base_config, f)

    for outlier_num in range(0, 13):
        base_config["detector_options"]["downsample_to_this_many_points"] = 18
        base_config["detector_options"]["add_this_many_outliers"] = int(outlier_num)
        for i in range(n_examples):
            new_dir = "o_%02d_s_%02d_ex%02d/" % (base_config["detector_options"]["add_this_many_outliers"], base_config["detector_options"]["downsample_to_this_many_points"], i)
            os.system("mkdir %s -p" % new_dir)
            with open("%s/config.yaml" % new_dir, 'w') as f:
                yaml.dump(base_config, f)