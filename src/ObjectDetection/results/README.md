Pose Estimation Results
=======

This directory contains scripts that fill the directory with test results from
the pose estimation methods, using data from the `../data` directory.

The results exists in a tree, organized by folder:

- At the top level, each folder is a method.
- Below each method is a folder for each of a set of configurations.
- Below each configuration is a config.yaml that represents the method configuration,
and the set of class and test instance folders that the method has been run on.
- In each test instance folder are:
-- "output.yaml", which contains the estimator output.
-- "summary.yaml", which contains metadata including error from ground truth,
the path to the original data directory used, and checksums of all input files
(including configuration), so that it's possible to detect when the results are old.