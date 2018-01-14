#!/usr/bin/env python

import os, sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import signal
import re
import hashlib
import argparse
from math import atan2

from director import transformUtils
from director.thirdparty import transformations

# Make sure you have a drakevisualizer open when running this, so you can see
# the vis output.

DATA_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/GlobalPoseEstimation/data/'

TARGET_CLASSES = ["crop_model_0.020"] # leave empty for all targets
#TARGET_CLASSES = ["crop_0.250"] # leave empty for all targets
#INSTANCE_PATTERN = re.compile(".*")
INSTANCE_PATTERN = re.compile("2017-06-16-51_drill")

#TARGET_CLASSES = ["scene_scale"]
#INSTANCE_PATTERN = re.compile("cube_resampled")

#TARGET_CLASSES = ["model_scale"] # leave empty for all targets
#INSTANCE_PATTERN = re.compile("cube_resampled_[0-9][0-9]")

def sha256_checksum(filename, block_size=65536):
  ''' Returns SHA256 checksum of the specified file. '''
  sha256 = hashlib.sha256()
  with open(filename, 'rb') as f:
      for block in iter(lambda: f.read(block_size), b''):
          sha256.update(block)
  return sha256.hexdigest()


def test_if_needs_update(config_file, input_dir, output_dir):
  ''' Given a configuration YAML file, an input data directory,
  and an output results directory, returns True if the
  output directory needs to be re-generated (i.e. if the
  output directory is incomplete, if the configuration or
  input scene or models points have changed), and False if not. '''
  try:
    if not os.path.isdir(output_dir):
      return True
    if not os.path.isfile(output_dir + "/output.yaml"):
      return True
    if not os.path.isfile(output_dir + "/summary.yaml"):
      return True

    # Load up the summary and check the metadata against the
    # config, as well as the supplied input info
    summary_yaml = yaml.load(open(output_dir + "/summary.yaml"))
    config_yaml = yaml.load(open(config_file))

    # Has config changed?
    if config_yaml != summary_yaml["config"]:
      print "Re-running dir %s because config has changed." % input_dir
      return True

    # Have input scene or model or ground truth checksums changed?
    scene_cloud_sha256 = sha256_checksum("%s/scene_cloud.vtp" % input_dir)
    model_cloud_sha256 = sha256_checksum("%s/model_cloud.vtp" % input_dir)
    ground_truth_sha256 = sha256_checksum("%s/ground_truth.yaml" % input_dir)

    if scene_cloud_sha256 != summary_yaml["scene_cloud_sha256"]:
      print "Re-running dir %s because scene cloud has changed."
      return True

    if model_cloud_sha256 != summary_yaml["model_cloud_sha256"]:
      print "Re-running dir %s because model cloud has changed."
      return True

    if ground_truth_sha256 != summary_yaml["ground_truth_sha256"]:
      print "Re-running dir %s because ground truth has changed."
      return True

  except Exception as e:
    print "Exception ", e, " caught while checking if update needed. Defaulted to yes."

  return False


def generate_run_mip_command(config_file, input_dir, output_dir, output_file):
  # Look up what kind of model we need, and form our filename path
  model_gt = yaml.load(open(input_dir + "/ground_truth.yaml"))
  model_file = DATA_DIR + "/" + model_gt["filename"] + ".decimated_50.urdf"
  q0 = model_gt["pose"][0]
  [q0.append(x) for x in model_gt["pose"][1]]

  print q0
  # Spit out a model configuration file for this script
  config_yaml = dict(
                   models = [
                      dict(urdf = model_file,
                           q0 = q0)
                    ])

  model_config_file = output_dir + "/model_config.yaml"
  with open(model_config_file, 'w') as f:
    yaml.dump(config_yaml, f)

  command = "run_mip_pose_estimator %s/scene_cloud.vtp %s %s %s" % (
    input_dir, model_config_file, config_file, output_file
  )
  return command

def run_method(method, config_file, input_dir, output_dir):
  ''' Runs the given method on a configuration YAML, an input data directory, and a
  directory to output the appropriate files to. '''
  os.system("mkdir -p " + output_dir)

  output_file = "%s/output.yaml" % (output_dir)

  if method == "mip":
    # This gets its own function, as it's pretty complex.
    command = generate_run_mip_command(config_file, input_dir, output_dir, output_file)
  elif method == "goicp":
    command = "run_goicp_pose_estimator %s/scene_cloud.vtp %s/model_cloud.vtp %s %s" % (
      input_dir, input_dir, config_file, output_file
    )
  elif method == "fgr":
    command = "run_fgr_pose_estimator %s/scene_cloud.vtp %s/model_cloud.vtp %s %s" % (
      input_dir, input_dir, config_file, output_file
    )
  elif method == "super4pcs":
    command = "run_super4pcs_pose_estimator %s/scene_cloud.vtp %s/model_cloud.vtp %s %s" % (
      input_dir, input_dir, config_file, output_file
    )
  elif method == "icp":
    command = "run_iterative_closest_point_estimator %s/scene_cloud.vtp %s/model_cloud.vtp %s %s" % (
      input_dir, input_dir, config_file, output_file)
  else:
    print "Detector type \"", method, "\" not recognized."
    exit(0)
  print "\n", command
  os.system(command)

  # Load up the output file and produce a score based on the accuracy of the TF
  output_yaml = yaml.load(open(output_file))
  output_pose = output_yaml["solutions"][0]["models"][0]["q"]
  output_trans = np.array(output_pose[0:3])
  if len(output_pose) == 7:
    output_quat = np.array(output_pose[3:7])
  else:
    output_quat = transformUtils.rollPitchYawToQuaternion(np.array(output_pose[3:6]))

  gt_yaml = yaml.load(open(input_dir + "/ground_truth.yaml"))

  target_trans = np.array(gt_yaml["pose"][0])
  target_quat = np.array(gt_yaml["pose"][1])

  trans_error = target_trans - output_trans

  d_1 = transformations.quaternion_multiply(output_quat, transformations.quaternion_inverse(target_quat))
  d_2 = transformations.quaternion_multiply(-output_quat, transformations.quaternion_inverse(target_quat))
  angle_error = min( 2*atan2(np.linalg.norm(d_1[1:]), d_1[0]), 2*atan2(np.linalg.norm(d_2[1:]), d_2[0]) )
  print "GT: ", target_trans, ", ", target_quat
  print "Estimated: ", output_trans, ", ", output_quat
  print "Trans error: ", list(trans_error)
  print "Angle error: ", angle_error

  # Collect those results into an appropriately named results file, along with the downsampling params used, and 
  # save it out.
  final_results_file = "%s/summary.yaml" % (output_dir)

  scene_cloud_sha256 = sha256_checksum("%s/scene_cloud.vtp" % input_dir)
  model_cloud_sha256 = sha256_checksum("%s/model_cloud.vtp" % input_dir)
  ground_truth_sha256 = sha256_checksum("%s/ground_truth.yaml" % input_dir)

  out_results = dict(
                 estimated = dict(
                  est_trans = output_trans.tolist(),
                  est_quat = output_quat.tolist()),
                 target = dict(
                  target_trans = target_trans.tolist(),
                  target_quat = target_quat.tolist()),
                 error = dict(
                  trans_error = trans_error.tolist(),
                  angle_error = float(angle_error)),
                 config = yaml.load(open(config_file)),
                 scene_cloud_sha256 = scene_cloud_sha256,
                 model_cloud_sha256 = model_cloud_sha256,
                 ground_truth_sha256 = ground_truth_sha256
                )

  with open(final_results_file, 'w') as f:
    yaml.dump(out_results, f)

def do_update(method, config_dir, rebuild_all):
  ''' Given a configuration file, iterates through the data
  directory, checks if each example instance has been run already
  and needs an update, and does the updates if necessary for each
  method that we know how to handle. '''

  # Iterate through available data...
  if not os.path.isdir(DATA_DIR):
    print "Data directory ", data_dir, " doesn't exist."
    exit(0)

  config_file = config_dir + "/config.yaml"

  class_names = next(os.walk(DATA_DIR))[1]
  for class_name in sorted(class_names):
    if class_name not in TARGET_CLASSES:
      print "Skipping class_name %s because it's not a target" % (class_name)
      continue

    # For each test instance...
    test_instances = next(os.walk(DATA_DIR + "/" + class_name))[1]
    for test_instance in sorted(test_instances):
      if not INSTANCE_PATTERN.match(test_instance):
        continue

      # Test if we need to update this instance at all
      input_dir = "%s/%s/%s/" % (DATA_DIR, class_name, test_instance)
      result_dir = "%s/%s/%s/" % (config_dir, class_name, test_instance)
      needs_update = test_if_needs_update(config_file, input_dir, result_dir)
      if rebuild_all or needs_update:
        print "\n\n***UPDATING INSTANCE %s***\n" % (test_instance)
        run_method(method, config_file, input_dir, result_dir)
        

if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  parser = argparse.ArgumentParser(description='Update pose estimation results.')
  parser.add_argument("-B", help="Rebuild all.", action='store_true')

  methods = ["fgr", "goicp", "mip", "super4pcs", "icp"]

  for method in methods:
    parser.add_argument("--" + method, help="Config to update, or \"all\"")

  args = parser.parse_args(sys.argv[1:])
  rebuild_all = args.B

  for method in methods:
    if hasattr(args, method):
      config_spec = getattr(args, method)
      if config_spec is None:
        continue
      elif config_spec == 'all':
        subdirs = next(os.walk("%s/" % method))[1]
        [ do_update(method, "%s/%s" % (method, subdir), rebuild_all) for subdir in subdirs ]
      else:
        config_pattern = re.compile(config_spec)
        subdirs = next(os.walk("%s/" % method))[1]
        for subdir in subdirs:
          if config_pattern.match(subdir):
            config_dir = "%s/%s/" % (method, subdir)
            do_update(method, config_dir, rebuild_all)