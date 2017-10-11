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
	calib_data[index]['camera-pose'] = dict()
	calib_data[index]['camera-pose']['quaternion'] = dict()
	calib_data[index]['camera-pose']['quaternion']['w'] = float(quat[0])
	calib_data[index]['camera-pose']['quaternion']['x'] = float(quat[1])
	calib_data[index]['camera-pose']['quaternion']['y'] = float(quat[2])
	calib_data[index]['camera-pose']['quaternion']['z'] = float(quat[3])
	calib_data[index]['camera-pose']['translation'] = dict()
	calib_data[index]['camera-pose']['translation']['x'] = float(pos[0])
	calib_data[index]['camera-pose']['translation']['y'] = float(pos[1])
	calib_data[index]['camera-pose']['translation']['Z'] = float(pos[2])


out_yaml_file_name = os.path.join(path_to_calibration_folder, "robot_camera_data.yaml")
with open(out_yaml_file_name, 'w') as outfile:
    yaml.dump(calib_data, outfile, default_flow_style=False)


###
# publish transforms for visp
###

## --- testing with random transforms -- ##

def randomTransform():
	transform = Transform()
	transform.translation.x = random.uniform(-1,1)
	transform.translation.y = random.uniform(-1,1)
	transform.translation.z = random.uniform(-1,1)
	transform.rotation.x    = random.uniform(-1,1)
	transform.rotation.y    = random.uniform(-1,1)
	transform.rotation.z    = random.uniform(-1,1)
	transform.rotation.w    = 1.0
	return transform

def applyCameraTransform(transform_in):
	transform = Transform()
	transform.translation.x = transform_in.translation.x
	transform.translation.y = transform_in.translation.y
	transform.translation.z = transform_in.translation.z + 0.5
	transform.rotation.x    = transform_in.rotation.x   
	transform.rotation.y    = transform_in.rotation.y   
	transform.rotation.z    = transform_in.rotation.z   
	transform.rotation.w    = transform_in.rotation.w   
	return transform

# gather world transforms
world_effector_transforms = []

for i in range(10):
	world_effector_transforms.append(randomTransform())

# gather camera transforms
camera_object_transforms = []

for i,v in enumerate(world_effector_transforms):
	camera_object_transforms.append(applyCameraTransform(v))

## --- testing with random transforms -- ##

#for index, value in enumerate(calib_data):



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