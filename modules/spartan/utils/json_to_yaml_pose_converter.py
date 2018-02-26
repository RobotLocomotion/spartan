import json
from pprint import pprint
import os

import spartan.utils.utils as spartanUtils

stored_poses_json_source = os.path.join(spartanUtils.getSpartanSourceDir(), 'models', 'iiwa', 'director','stored_poses.json')
data_in = json.load(open(stored_poses_json_source))
#pprint(data_in)

stored_poses_yaml_destination = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects', 'station_config','RLG_iiwa_1','stored_poses.yaml')
data_out = dict()

joint_names =  ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
data_out['header'] = dict()
data_out['header']['robot'] = 'kuka_iiwa'
data_out['header']['num_joints'] = 7
data_out['header']['joint_names'] = joint_names

for group in data_in:
	data_out[str(group)] = dict()
	print "Group is", group
	for pose in data_in[group]:
		pose_name = pose['name']
		print "pose_name is", pose_name
		data_out[str(group)][str(pose_name)] = []
		for k in joint_names:
			joint_angle = pose['joints'][k]
			data_out[group][str(pose_name)].append(joint_angle)


print stored_poses_yaml_destination
spartanUtils.saveToYaml(data_out, stored_poses_yaml_destination)
