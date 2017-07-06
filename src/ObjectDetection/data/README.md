Pose Estimation Test Set
=======

This directory contains scripts that fill the directory with test datasets
for pose estimation algorithms.

The data exists in a tree, organized by folder:

- At the top level, each folder is a class of data, fundamentally different
in some way than the other sets..
- Each class folder contains:
  - A `details.yaml` file containing a "name" field that names the class,
  and a "details" field that contains some generation parameters and brief details.
  - A set of test instance folders, each of which is a single test item.
  - Each test instance folder contains:
    - "scene_cloud.[vtp, ply]", the scene cloud that methods may consume 
    - "model_cloud.[vtp, ply]", the model cloud that methods may consume
    - In some cases, extra files used in the generation of the above.
    - "ground_truth.yaml", which contains:
      - The ground truth transformation between the model and scene, as translation [x,y,z] and quaternion [w,x,y,z]
      - The path to the original model used to generate the model cloud, relative to the root of this data directory.