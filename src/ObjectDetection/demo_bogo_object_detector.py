#!/usr/bin/env python

import os

# Make sure you have a drakevisualizer open when running this, so you can see
# the vis output.

# Make sure we have a tmp directory to spit data into.
os.system("mkdir -p tmp")

# Generate the sample point cloud using a pre-written config file
os.system("run_point_cloud_generator config/point_cloud_generator_config.yaml tmp/cube_samples.pcd")

# Run the bogo detector using the sampled data we just generated
os.system("run_bogo_object_detector config/bogo_detector_ex.yaml tmp/cube_bogo_fit.out")
