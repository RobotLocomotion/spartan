#!/usr/bin/env python 

# Usage: (make sure to source ros's setup.bash in each terminal)
# roscore
#
# rosrun visp_hand2eye_calibratiosp_hand2eye_calibration_calibrator 
#
# directorPython publishCalibrationDataToVisp.py

import rospy
import yaml
import time
import random
import os
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera

sys.path.append('../LabelFusion/modules/labelfusion')
from cameraposes import CameraPoses
from director import transformUtils


path_to_calibration_folder = os.path.join(os.getcwd(),"data/20171011-145541")

###
# generate camera poses to match utimes of robot poses
##

# create CameraPoses object from timestamped posegraph of camera
posegraph_file = os.path.join(path_to_calibration_folder,"posegraph.posegraph")
print posegraph_file
cameraposes = CameraPoses(posegraph_file)

# read in available robot data
in_yaml_file_name = os.path.join(path_to_calibration_folder, "robot_data.yaml")
with open(in_yaml_file_name, 'r') as f:
    calib_data = yaml.load(f)


# augment data with matched camera poses
# NOTE: camera poses are defined relative to first camera frame, hence the first one will look unintialized
for index, value in enumerate(calib_data):
	transform = cameraposes.getCameraPoseAtUTime(value['utime'])
	(pos, quat) = transformUtils.poseFromTransform(transform)
	calib_data[index]['camera_frame'] = dict()
	calib_data[index]['camera_frame']['quaternion'] = dict()
	calib_data[index]['camera_frame']['quaternion']['w'] = float(quat[0])
	calib_data[index]['camera_frame']['quaternion']['x'] = float(quat[1])
	calib_data[index]['camera_frame']['quaternion']['y'] = float(quat[2])
	calib_data[index]['camera_frame']['quaternion']['z'] = float(quat[3])
	calib_data[index]['camera_frame']['translation'] = dict()
	calib_data[index]['camera_frame']['translation']['x'] = float(pos[0])
	calib_data[index]['camera_frame']['translation']['y'] = float(pos[1])
	calib_data[index]['camera_frame']['translation']['z'] = float(pos[2])


out_yaml_file_name = os.path.join(path_to_calibration_folder, "robot_camera_data.yaml")
with open(out_yaml_file_name, 'w') as outfile:
    yaml.dump(calib_data, outfile, default_flow_style=False)


###
# publish transforms for visp
###

# gather camera transforms
camera_object_transforms = []
world_effector_transforms = []

## --- testing with random transforms -- ##
def randomTransform():
	transform = Transform()
	transform.translation.x = random.uniform(-1,1)
	transform.translation.y = random.uniform(-1,1)
	transform.translation.z = random.uniform(-1,1)
	transform.rotation.x    = random.uniform(-1,1)
	transform.rotation.y    = random.uniform(-1,1)
	transform.rotation.z    = random.uniform(-1,1)
	transform.rotation.w    = random.uniform(-1,1)
	quat_norm = math.sqrt(transform.rotation.x**2 + transform.rotation.y**2 + transform.rotation.z**2 + transform.rotation.w**2)
	transform.rotation.x    /= quat_norm
	transform.rotation.y    /= quat_norm
	transform.rotation.z    /= quat_norm
	transform.rotation.w    /= quat_norm
	return transform

def rosTransformToVTKTransform(ros_transform):
	pos = []
	pos.append(ros_transform.translation.x)
	pos.append(ros_transform.translation.y)
	pos.append(ros_transform.translation.z)
	quat = []
	quat.append(ros_transform.rotation.w)
	quat.append(ros_transform.rotation.x)
	quat.append(ros_transform.rotation.y)
	quat.append(ros_transform.rotation.z)
	vtk_transform = transformUtils.transformFromPose(pos, quat)
	return vtk_transform


def vtkTransformToROSTransform(vtk_transform):
	ros_transform = Transform()
	(pos, quat) = transformUtils.poseFromTransform(transform)
	ros_transform.translation.x = pos[0]
	ros_transform.translation.y = pos[1]
	ros_transform.translation.z = pos[2]
	ros_transform.rotation.x    = quat[1]
	ros_transform.rotation.y    = quat[2]
	ros_transform.rotation.z    = quat[3]
	ros_transform.rotation.w    = quat[0]   
	return ros_transform

def constHandToCameraTransformVtk():
	pos = [0.0, 0.0, 0.5]
	quat = [1.0, 0.0, 0.0, 0.0]
	vtk_transform = transformUtils.transformFromPose(pos, quat)
	print vtk_transform
	return vtk_transform
 
def getCameraToWorld(world_to_hand_ros_transform_in):
	world_to_hand_transform_vtk = rosTransformToVTKTransform(world_to_hand_ros_transform_in)
	hand_to_camera_transform_vtk = constHandToCameraTransformVtk()
	#objectToWorld = transformUtils.concatenateTransforms([objectToFirstFrame, firstFrameToWorldTransform])   # example
	world_to_camera_transform_vtk = transformUtils.concatenateTransforms([world_to_hand_transform_vtk, hand_to_camera_transform_vtk])
	camera_to_world_transform_vtk = world_to_camera_transform_vtk.GetLinearInverse()
	return vtkTransformToROSTransform(camera_to_world_transform_vtk)

for i in range(10):
	world_effector_transforms.append(randomTransform())

for i,v in enumerate(world_effector_transforms):
	camera_object_transforms.append(getCameraToWorld(v))
## --- testing with random transforms -- ##

# def yamlEntryToROSGeometryTransform(yaml_entry):
# 	transform = Transform()
# 	print yaml_entry['translation']
# 	print yaml_entry['quaternion']
# 	if 'y' not in yaml_entry['quaternion']:
# 		yaml_entry['quaternion']['y'] = yaml_entry['quaternion']['y`'] 

# 	transform.translation.x = yaml_entry['translation']['x']
# 	transform.translation.y = yaml_entry['translation']['y']
# 	transform.translation.z = yaml_entry['translation']['z']
# 	transform.rotation.w    = yaml_entry['quaternion']['w']
# 	transform.rotation.x    = yaml_entry['quaternion']['x']
# 	transform.rotation.y    = yaml_entry['quaternion']['y']
# 	transform.rotation.z    = yaml_entry['quaternion']['z']
# 	return transform

# for index, value in enumerate(calib_data):
# 	print index
# 	world_effector_transforms.append(yamlEntryToROSGeometryTransform(value['hand_frame']))
# 	camera_object_transforms.append(yamlEntryToROSGeometryTransform(value['camera_frame']))


# init node and publishers
rospy.init_node('visp_calibration_data_publisher')
world_effector_pub = rospy.Publisher('/world_effector', Transform, queue_size=1000)
camera_object_pub  = rospy.Publisher('/camera_object', Transform, queue_size=1000)

# hack: need to sleep between advertising topic and publishing topic, otherwise
# visp may not receive all of them
time.sleep(2)

# publish Transfroms

for i in range(len(world_effector_transforms)):
	world_effector_pub.publish(world_effector_transforms[i])
	camera_object_pub.publish(camera_object_transforms[i])

# hack: need to sleep after publishing all transforms, otherwise visp may not
# be ready when it is asked to compute the hand-eye ("effector_camera") transform
time.sleep(2)

# initialize service proxy for the compute_effector_camera service
rospy.wait_for_service('/compute_effector_camera')
service_proxy = rospy.ServiceProxy('/compute_effector_camera', compute_effector_camera)
try:
  reply = service_proxy()
  print reply
except rospy.ServiceException as exc:
  print("Service did not process request: " + str(exc))

rospy.spin() # not actually necessary, just blocks