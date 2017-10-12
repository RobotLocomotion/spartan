#!/usr/bin/env directorPython

import argparse
import os
import yaml
import os

import spartan.perception.handeyecalibration as handeyecalibration


if __name__=="__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("-d", "--data_file", type=str, default="robot_data.yaml", required=False, help="name of the data file")
	parser.add_argument("-c", "--camera_pose_file", type=str, default='posegraph.posegraph')
	args = parser.parse_args()
	dataFilename = args.data_file
	saveDataFilename = "camera_poses_and_" + dataFilename

	currentDirectory = os.getcwd()
	posegraph_file = os.path.join(currentDirectory, args.camera_pose_file)
	robot_data_file = os.path.join(currentDirectory, dataFilename)
	save_data_file = os.path.join(currentDirectory, saveDataFilename)
	handeyecalibration.processCalibrationData(posegraph_file, robot_data_file, save_data_file)