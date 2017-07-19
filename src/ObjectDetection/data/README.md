Pose Estimation Test Set
=======

This directory contains scripts that fill the directory with test datasets
for pose estimation algorithms.

How to generate data
=======
1) Download the [LabelFusion dataset](http://lab.protos.homenet.org:8000/) to
somewhere on your filesystem. Either download the `.tar.gz` file,
or download the entire `data` directory.
2) Edit `create_test_class_from_coral_data.py` to point `CORL_DATA_DIR`
and `MODELS_ROOT_DIR` to the `./logs_test` and `./` paths within the labelfusion
dataset. Modify the `params` dict to your liking: `scene_resample_spacing` and
`model_resample_spacing` control the spacing (in meters) at which models are
resampled to generate the test data. `scene_crop_width` specifies the size
of the bounding box around the object location that data is pulled from if
`crop_around_model` is `False`, and specifies the max distance from the object
surface that data is pulled from if `crop_around_model` is True.
3) Run `python create_test_class_from_coral_data.py` and confirm that data is
generated! It might take a while if you are using the entire LabelFusion dataset.


Data structure
=========
Once generated, the data will exist in a tree, organized by folder:

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