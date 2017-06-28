#!/usr/bin/env python

import os, sys
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import signal
import re
import hashlib

from director import transformUtils

# Make sure you have a drakevisualizer open when running this, so you can see
# the vis output.

DATA_DIR = os.environ['SPARTAN_SOURCE_DIR'] + '/src/ObjectDetection/data/'

TARGET_CLASSES = ["crop_0.250"] # leave empty for all targets
INSTANCE_PATTERN = re.compile(".*")


def sha256_checksum(filename, block_size=65536):
    sha256 = hashlib.sha256()
    with open(filename, 'rb') as f:
        for block in iter(lambda: f.read(block_size), b''):
            sha256.update(block)
    return sha256.hexdigest()


def test_if_needs_update(config_file, input_dir, output_dir):
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



def run_fgr(config_file, input_dir, output_dir):
  os.system("mkdir -p " + output_dir)

  output_file = "%s/output.yaml" % (output_dir)
  
  command = "run_fgr_detector %s/scene_cloud.vtp %s/model_cloud.vtp %s %s" % (
      input_dir, input_dir, config_file, output_file
    )
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
  angle_error = np.arccos( (np.dot(output_quat, target_quat)) / (np.linalg.norm(target_quat) * np.linalg.norm(output_quat)))
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

def do_update(config_dir):
  # Iterate through available data...
  if not os.path.isdir(DATA_DIR):
    print "Data directory ", data_dir, " doesn't exist."
    exit(0)

  config_file = config_dir + "/config.yaml"

  class_names = next(os.walk(DATA_DIR))[1]
  for class_name in sorted(class_names):
    if class_name not in TARGET_CLASSES:
      print "Skipping class_name %s because it's not a target" % (class_name)

    # For each test instance...
    test_instances = next(os.walk(DATA_DIR + "/" + class_name))[1]
    for test_instance in sorted(test_instances):
      if not INSTANCE_PATTERN.match(test_instance):
        continue

      # Test if we need to update this instance at all
      input_dir = "%s/%s/%s/" % (DATA_DIR, class_name, test_instance)
      result_dir = "%s/%s/%s/" % (config_dir, class_name, test_instance)
      needs_update = test_if_needs_update(config_file, input_dir, result_dir)
      if (needs_update):
        print "\n\n***UPDATING INSTANCE %s***\n" % (test_instance)
        run_fgr(config_file, input_dir, result_dir)

if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  if len(sys.argv) != 2:
    print "Usage: python <script name> <config dir to update, or 'all'"
    exit(0)

  config_dir = sys.argv[1]

  if config_dir == 'all':
    subdirs = next(os.walk("."))[1]
    [ do_update(subdir) for subdir in subdirs ]

  else:
    if not os.path.isdir(config_dir) or not os.path.isfile(config_dir + "/config.yaml"):
      print "%s is not a valid configuration subdirectory -- please create it and give it" % config_dir
      print " a config.yaml file."
      exit(0)

    do_update(config_dir)
