Pose Estimation Results
=======

This directory contains scripts that fill the directory with test results from
the pose estimation methods, using data from the [`../data` directory.](../data/README.md)

Results output structure
========
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

How to generate results
=========
The `run_methods_on_data.py` script is the core utility for running pose estimation
methods on the test set. It includes logic to avoid re-running estimation methods unless
the configuration or input data has changed (or unless forced with `-B`). Open it and
tweak the `TARGET_CLASSES` and `INSTANCE_PATTERN` to target any particular test
classes and instances that you are interested in (e.g. only class `crop_model_0.0250`,
which was generated with data cropped to only points within 0.0250m of the model surface,
and instance pattern `.*drill`, which only targets instances that include the drill model.
The script should be invoked with at least one argument `--<method_name>=<config name>`,
where `<method_name>` is one of the known methods (mip, super4pcs, fgr) and `<config name>` is
either `all` or a specific existing configuration subdirectory name.