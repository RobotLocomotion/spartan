Pose Estimation
=======

This directory contains development and testing scripts for object
pose estimation in point clouds.


Object Pose Estimation Technique Comparison
==================

We defined a 'Pose Estimation Problem' which contains all the information needed
to evaluate a pose estimation method on a point cloud. It is defined as a YAML file
that contains:
  - "points": A pointcloud, as a path to a VTP file. The frame of reference
         of these points may be camera, or some other arbitrary frame, but all
         object poses will be reported in the same frame as this point cloud.
  - A list of models to fit. Each model contains:
    - A path to a VTP that describes the model.
    - The ground truth pose of the model in the point cloud, as a floating base for
      the obj. Floating base coordinates may use both
      xyz-rpy and xyz-quaternion.


To compare pose estimation techniques, we define a generic `Model Pose Estimator` from 
which comparable techniques derive, which performs object pose estimation given a
Pose Estimation Problem. It consumes a Pose Estimation Problem and any additional arguments
desired by the method, and produces a 'Pose Estimation Result' YAML file that contains:
  - A list of models, by filename, and their estimated poses.