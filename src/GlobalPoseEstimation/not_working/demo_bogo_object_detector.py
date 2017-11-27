#!/usr/bin/env python

import os

# Make sure you have a drakevisualizer open when running this, so you can see
# the vis output.

# Make sure we have a tmp directory to spit data into.
os.system("mkdir -p tmp")

# Generate the sample point cloud using a pre-written config file
os.system("run_point_cloud_generator config/point_cloud_generator_config.yaml tmp/cube_samples.pcd tmp/cube_sample_gt.yaml")

# Run the bogo detector using the sampled data we just generated
os.system("run_bogo_object_detector config/bogo_detector_ex.yaml tmp/cube_bogo_fit.yaml")

# Parse ground truth and compare to the output
import yaml
import numpy as np
gt_yaml = yaml.load(open("tmp/cube_sample_gt.yaml"))
fit_yaml = yaml.load(open("tmp/cube_bogo_fit.yaml"))

models_and_gt_pose = {}
for model in gt_yaml:
	models_and_gt_pose[model["urdf"]] = np.array(model["q0"])

# take first fit in the file, in case their are multiple
models_and_fit_pose = {}
for model in fit_yaml[0]["models"]:
	models_and_fit_pose[model["urdf"]] = np.array(model["q"])

for name in models_and_gt_pose.keys():
	if name not in models_and_fit_pose.keys():
		print "Did not find model name ", name, " in fit models."
	else:
		error = models_and_fit_pose[name] - models_and_gt_pose[name]
		print "Squared error ", np.dot(error.transpose(), error), " for model ", name