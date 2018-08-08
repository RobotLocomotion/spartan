#!/usr/bin/python

import os
import rospy
import time
import argparse

from fusion_server.fusion import FusionServer
import fusion_server.tsdf_fusion as tsdf_fusion

import spartan.utils.utils as spartanUtils

import batch_extract_and_fuse_all_scenes


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--log_dir", type=str, required=True)
    
    args = parser.parse_args()

    batch_extract_and_fuse_all_scenes.extract_and_fuse_single_scene(args.log_dir, downsample=True)

