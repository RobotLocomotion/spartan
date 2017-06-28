Pose Estimation Test Set
=======

This directory contains scripts that fill the directory with test datasets
for pose estimation algorithms.

The data exists in a tree, organized by folder:

- At the top level, each folder is a class of data, fundamentally different
in some way than the other sets. 
- A class folder contains a `details.yaml` file containing a "name" field that names the class,
and a "details" field that contains details.
- The class folder contains a set of test instance folders, each of which is a single test item.
- Each test instance folder contains:
-- "scene_cloud.vtp", the scene cloud that methods may consume 
-- "model_cloud.vtp", the model cloud that methods may consume
-- "model_mesh.obj", the model mesh that methods may consume 
-- "ground_truth.yaml", the ground truth transformation between the model and scene