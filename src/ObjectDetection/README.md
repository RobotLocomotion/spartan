Pose Estimation
=======

This directory contains development and testing scripts for single-shot object
pose estimation in point clouds. It includes the source code used for
[Globally Optimal Object Pose Estimation in Point Clouds with
Mixed Integer Programming]
(http://groups.csail.mit.edu/robotics-center/public_papers/Izatt17.pdf).

Object Pose Estimation Technique Comparison
==================

We regard the object 'Pose Estimation Problem' as one in which, given a point cloud
and a model description, one must find the optimal configuration of the model to
explain that point cloud.

For automation purposes, we assume that a pose estimation utility can be operated
by a simple command line interface accepting, at minimum, a `vtp`-format point cloud
and producing an output YAML file that contains a description of the predicted object
pose.

The output file should be a `.yaml`-format file, and it should have the structure
specified below:

```
config:
  <any method-specific configuration that should be saved>
solutions:
  - models:
    - model: <model filename>
      q: [<configuration of model>]
    history:
      <any solve history information you wish to save for this solution>
```

That is, it should specify the configuration used to generate the results,
as well as a list of solutions (possibly multiple solutions), each of which
specifies a model filename (it can be a '.vtp' geometry file, a '.obj',
a '.urdf'...) and its configuration, along with any solve-history info that
might be useful.

A few popular global pose estimation methods (Fast Global Registration, Super4PCS) are
wrapped in this way in `src.`

How to get this working
=================
1) Make sure Spartan is building this directory, and that utilities like `run_<method>_detector.cpp`
are on your path.
2) Follow the directions in the [data directory](data/README.md) to generate the test dataset.
3) Follow the instructions in the [results directory](results/README.md) to run the methods on the
test dataset, or manually invoke the method on data of your choosing. Example configuration scripts
that can be fed to each method are available both in subdirectories of `results`, and also in `config.`
(The ones in `results` are more likely to be up to date.)
