#!/usr/bin/env python

# Simulates an IIWA with a Schunk gripper, using a supplied
# configuration file to set up the environment.
# (See, e.g., rlg_simulation/config/iiwa_workstation_with_object.yaml.)
#
# Accept IIWA commands over LCM and produces IIWA state over LCM, mocking
# the current IIWA driver.
# Accepts Schunk commands over ROS and produces Schunk state over ROS,
# mocking the current Schunk driver.
# Generates raw RGB and Depth data, using a specified depth camera
# configuration profile.
# Note that the RGB / RGB camera info currently published are wrong in at
# least two ways:
#   - 1) The RGB image is published from the Depth camera extrinsic location.
#   - 2) The RGB camera frame <-> hand frame are specified in
#        camera_info.yaml but probably never published out to the rest of the
#        ROS tf system. So either figure out how to tell the openni stack to
#        do that, or do it yourself. (Ask Kuni)
# 
# Arguments:
#   config: Configuration file
#   rate: Desired real-time rate (will not run faster, might run slower)
# Press q to quit and r to restart the simulation.

import argparse
from copy import deepcopy
import math
import numpy as np
import os
import re
import rospy
import pybullet
import threading
import time
import yaml

# Comms with Arm
import lcm
from drake import lcmt_iiwa_command, lcmt_iiwa_status ,lcmt_robot_state

# Comms with hand
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils
import spartan.utils.cv_utils as cvUtils
import wsg50_msgs.msg

# Comms from camera
import std_msgs.msg
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Visualization to Director
import pydrake
import pydrake.rbtree
import RemoteTreeViewerWrapper_pybind as Rtv

# Loading URDFs from ROS package paths
from director.packagepath import PackageMap


# TODO(gizatt): Slowly port the relevant parts of this to the
# simulation configuration file. For now, tweaks can be made
# in this file as needed.

# The controlled robot URDF, including both the IIWA *and*
# the gripper. (I haven't figured out how to programmatically
# attach the hand onto the gripper in Pybullet yet. Easier to
# specify all-in-one and programmatically separate their joints...)
kIiwaUrdf = "${SPARTAN_SOURCE_DIR}/models/iiwa/iiwa_description/iiwa14_schunk_gripper.urdf"

# IIWA joints that IIWA commands will affect
IIWA_CONTROLLED_JOINTS = [
    "iiwa_joint_1",
    "iiwa_joint_2",
    "iiwa_joint_3",
    "iiwa_joint_4",
    "iiwa_joint_5",
    "iiwa_joint_6",
    "iiwa_joint_7",
]
# Max force the joints will use to track their setpoint
# in position control mode.
IIWA_MAX_JOINT_FORCES = [
    10000.,
    10000.,
    10000.,
    10000.,
    10000.,
    10000.,
    10000.
]

# Schunk joints that Schunk commands will affect
SCHUNK_CONTROLLED_JOINTS = [
    "wsg_50_base_joint_gripper_left",
    "wsg_50_base_joint_gripper_right",
]
# Schunk finger link names
#SCHUNK_FINGER_LINKS = [
#    
#]


# Rendering range of the camera
# defaults for the carmine
IIWA_DEPTH_CAMERA_MIN_DISTANCE = 0.35 
IIWA_DEPTH_CAMERA_MAX_DISTANCE = 3.0

IIWA_RGB_CAMERA_MIN_DISTANCE = 0.1
IIWA_RGB_CAMERA_MAX_DISTANCE = 10.

CAMERA_FORWARD_VEC = [0,0,1]
CAMERA_UP_VEC = [0,1,0]



def load_pybullet_from_urdf_or_sdf(inp_path, position = [0, 0, 0], quaternion = [0, 0, 0, 1], fixed = True, packageMap = None):
    full_path = os.path.expandvars(inp_path)

    ext = full_path.split(".")[-1]
    if ext == "urdf":
        print "Loading ", full_path, " as URDF in pos ", position

        if packageMap is not None:
            # load text of URDF, resolve package URLS to absolute paths,
            # and save it out to a temp directory
            with open(full_path, 'r') as urdf_file:
                full_text = urdf_file.read()
                with open("/tmp/resolved_urdf.urdf", 'w') as out_file:
                    # Find package references
                    matches = re.finditer(r'package:\/\/[^\/]*\/', full_text)
                    curr_ind = 0
                    # Replace them in output file with the global path
                    for match in matches:
                        out_file.write(full_text[curr_ind:match.start()])
                        out_file.write(packageMap.resolveFilename(match.group(0)))
                        curr_ind = match.end()
                    out_file.write(full_text[curr_ind:-1])
            full_path = "/tmp/resolved_urdf.urdf"

        return pybullet.loadURDF(full_path, basePosition=position, baseOrientation=quaternion, useFixedBase=fixed)
    elif ext == "sdf":
        print "Loading ", full_path, " as SDF in pos ", position
        return pybullet.loadSDF(full_path)
    else:
        print "Unknown extension in path ", full_path, ": ", ext
        exit(-1)


def load_drake_from_urdf_or_sdf(inp_path):
    full_path = os.path.expandvars(inp_path)
    return pydrake.rbtree.RigidBodyTree(full_path, floating_base_type=pydrake.rbtree.kRollPitchYaw)

class RgbdCameraMetaInfo():
    def __init__(self, camera_serial, linkNameToJointIdMap):
        rgb_info_file = "${SPARTAN_SOURCE_DIR}/src/catkin_projects/camera_config/data/%s/master/rgb_camera_info.yaml" % camera_serial
        depth_info_file = "${SPARTAN_SOURCE_DIR}/src/catkin_projects/camera_config/data/%s/master/depth_camera_info.yaml" % camera_serial
        overall_info_file = "${SPARTAN_SOURCE_DIR}/src/catkin_projects/camera_config/data/%s/master/camera_info.yaml" % camera_serial

        self.rgb_info_msg = self.populateCameraInfoMsg(rgb_info_file)
        self.depth_info_msg = self.populateCameraInfoMsg(depth_info_file)

        info_yaml = yaml.load(open(os.path.expandvars(overall_info_file), 'r'))
        # Process overall camera info:
        self.rgb_extrinsics = self.processCameraExtrinsicsYaml(info_yaml["rgb"]["extrinsics"], linkNameToJointIdMap)
        self.depth_extrinsics = self.processCameraExtrinsicsYaml(info_yaml["depth"]["extrinsics"], linkNameToJointIdMap)

        self.camera_serial_number = camera_serial
        self.topics = RgbdCameraMetaInfo.getCameraPublishTopics(self.camera_serial_number)
        self.frames = RgbdCameraMetaInfo.getCameraFrames(self.camera_serial_number)


        if 'min_range' in info_yaml['depth']:
            self.depth_min_range = info_yaml['depth']['min_range']
        else:
            self.depth_min_range = IIWA_DEPTH_CAMERA_MIN_DISTANCE

        if 'max_range' in info_yaml['depth']:
            self.depth_max_range = info_yaml['depth']['max_range']
        else:
            self.depth_max_range = IIWA_DEPTH_CAMERA_MAX_DISTANCE

        

    @staticmethod
    def populateCameraInfoMsg(info_filename):
        # Construct a CameraInfo msg
        info_yaml = yaml.load(open(os.path.expandvars(info_filename), 'r'))
        info_msg = sensor_msgs.msg.CameraInfo()
        info_msg.width = info_yaml["image_width"]
        info_msg.height = info_yaml["image_height"]
        info_msg.distortion_model = info_yaml["distortion_model"]
        info_msg.D = info_yaml["distortion_coefficients"]["data"]
        info_msg.K = info_yaml["camera_matrix"]["data"]
        info_msg.R = info_yaml["rectification_matrix"]["data"]
        info_msg.P = info_yaml["projection_matrix"]["data"]
        return info_msg

    @staticmethod
    def processCameraExtrinsicsYaml(extrinsics_yaml, linkNameToJointIdMap):
        extrinsics_dict = {"parent_joint_id": 0,
                           "pose_xyz": [0., 0., 0.],
                           "pose_quat": [1., 0., 0., 0.]}
        extrinsics_dict["parent_joint_id"] = linkNameToJointIdMap[extrinsics_yaml["reference_link_name"]]
        tf = extrinsics_yaml["transform_to_reference_link"]
        extrinsics_dict["pose_xyz"][0] = tf["translation"]["x"]
        extrinsics_dict["pose_xyz"][1] = tf["translation"]["y"]
        extrinsics_dict["pose_xyz"][2] = tf["translation"]["z"]
        
        quat = spartanUtils.getQuaternionFromDict(tf)
        extrinsics_dict["pose_quat"][0] = quat["w"]
        extrinsics_dict["pose_quat"][1] = quat["x"]
        extrinsics_dict["pose_quat"][2] = quat["y"]
        extrinsics_dict["pose_quat"][3] = quat["z"]
        return  extrinsics_dict

    @staticmethod
    def getCameraPublishTopics(camera_serial_number):
        topics = dict()
        topics['RGB_TOPIC'] = "/camera_%s/rgb/image_raw" % camera_serial_number
        topics['DEPTH_TOPIC'] = "/camera_%s/depth/image_raw" % camera_serial_number
        topics['RGB_INFO_TOPIC'] = "/camera_%s/rgb/camera_info" % camera_serial_number
        topics['DEPTH_INFO_TOPIC'] = "/camera_%s/depth/camera_info" % camera_serial_number
        return topics

    @staticmethod
    def getCameraFrames(camera_serial_number):
        frames = dict()
        frames['RGB_FRAME'] = "camera_%s_rgb_optical_frame" % camera_serial_number
        frames['DEPTH_FRAME'] = "camera_%s_depth_optical_frame" % camera_serial_number

        return frames

    

class IiwaRlgSimulator():
    ''' 
    This class implements a simulation of the Kuka IIWA + Schunk WSG 50
    + random other objects simulation in Pybullet. 

    Its constructor subscribes to the appropriate input channels:
        - LCM channel `IIWA_COMMAND`, type `lcmt_iiwa_command`, for joint position
          commands. (Torque mode not supported, but it would be easy to switch.)
        - Ros topic `/schunk_driver/schunk_wsg_command`, type
          wsg50_msgs::WSG_50_command for position setpoing and max force
    
    ResetSimulation() sets up the simulation.

    RunSim run a sim until (R)eset or (Q)uit from the sim gui.
    '''

    def __init__(self, config, timestep, rate, rgbd_noise=0.005, rgbd_normal_limit=0.05, rgbd_projector_baseline=0.1, camera_serial_number="carmine_1"):
        self.config = config
        self.timestep = timestep
        self.rate = rate
        self.camera_serial_number = camera_serial_number

        self.packageMap = PackageMap()
        self.packageMap.populateFromEnvironment(["ROS_PACKAGE_PATH"])

        self.iiwa_command_lock = threading.Lock()
        self.last_iiwa_position_command = [0.] * len(IIWA_CONTROLLED_JOINTS)

        self.schunk_command_lock = threading.Lock()
        self.last_schunk_position_command = [0.] * len(SCHUNK_CONTROLLED_JOINTS)
        self.last_schunk_torque_command = [0.] * len(SCHUNK_CONTROLLED_JOINTS)

        # Set up IIWA command subscriber over LCM
        self.lc = lcm.LCM()
        self.iiwa_command_sub = self.lc.subscribe("IIWA_COMMAND", self.HandleIiwaCommand)        

        # Set up Schunk command subscriber on ROS
        self.schunk_command_sub = rosUtils.SimpleSubscriber("/schunk_driver/schunk_wsg_command", wsg50_msgs.msg.WSG_50_command, self.HandleSchunkCommand)
        self.schunk_command_sub.start(queue_size=1)
        self.schunk_status_publisher = rospy.Publisher("/schunk_driver/schunk_wsg_status", wsg50_msgs.msg.WSG_50_state, queue_size=1)

        # Set up image publishing
        camera_topics = RgbdCameraMetaInfo.getCameraPublishTopics(self.camera_serial_number)
        self.rgb_publisher = rospy.Publisher(camera_topics['RGB_TOPIC'], sensor_msgs.msg.Image, queue_size=1)
        self.depth_publisher = rospy.Publisher(camera_topics['DEPTH_TOPIC'], sensor_msgs.msg.Image, queue_size=1)
        self.rgb_info_publisher = rospy.Publisher(camera_topics['RGB_INFO_TOPIC'], sensor_msgs.msg.CameraInfo, queue_size=1)
        self.depth_info_publisher = rospy.Publisher(camera_topics['DEPTH_INFO_TOPIC'], sensor_msgs.msg.CameraInfo, queue_size=1)
        self.cv_bridge = CvBridge()

        self.rgbd_projector_baseline = rgbd_projector_baseline
        self.rgbd_normal_limit = rgbd_normal_limit
        self.rgbd_noise = rgbd_noise

        # Set up a Remote Tree Viewer wrapper to publish
        # object states
        self.rtv = Rtv.RemoteTreeViewerWrapper()

    def ResetSimulation(self):
        pybullet.resetSimulation()

        pybullet.setGravity(0,0,-9.81)
        pybullet.setTimeStep(self.timestep)

        # Read in configuration file
        config = yaml.load(open(self.config))

        if config["with_ground"] == True:
            self.ground_id = load_pybullet_from_urdf_or_sdf(os.environ["SPARTAN_SOURCE_DIR"] + "/build/bullet3/data/plane.urdf")
        else:
            self.ground_id = None

        # Load in the Kuka
        if config["robot"]:
            q0 = config["robot"]["base_pose"]
            position = q0[0:3]
            quaternion = pybullet.getQuaternionFromEuler(q0[3:6])
            self.kuka_id = load_pybullet_from_urdf_or_sdf(kIiwaUrdf, position, quaternion, True, self.packageMap)

            for obj_id in range(-1, pybullet.getNumJoints(self.kuka_id)):
                print "Dynamic info for body %d, %d:" % (self.kuka_id, obj_id)
                print pybullet.getDynamicsInfo(self.kuka_id, obj_id)
                pybullet.changeDynamics(self.kuka_id, obj_id, lateralFriction=0.9, spinningFriction=0.9, frictionAnchor=1)

        self.BuildJointNameInfo()
        self.BuildMotorIdList()

        # Models entry is a dictionary of model URDF strings
        model_dict = config["models"]
        self.object_ids = []

        # Add each model as requested
        self.rbts = []
        for instance in config["instances"]:
            q0 = instance["q0"]
            position = q0[0:3]
            quaternion = pybullet.getQuaternionFromEuler(q0[3:8])
            fixed = instance["fixed"]
            self.object_ids.append(load_pybullet_from_urdf_or_sdf(model_dict[instance["model"]], position, quaternion, fixed))
            # Report all friction properties
            for obj_id in range(-1, pybullet.getNumJoints(self.object_ids[-1])):
                print "Dynamic info for body %d, %d:" % (self.object_ids[-1], obj_id)
                print pybullet.getDynamicsInfo(self.object_ids[-1], obj_id)
                pybullet.changeDynamics(self.object_ids[-1], obj_id, lateralFriction=0.9, spinningFriction=0.9, frictionAnchor=0)

            self.rbts.append(load_drake_from_urdf_or_sdf(model_dict[instance["model"]]))

        # Set up camera info (which relies on having models loaded
        # so we know where to mount the camera)
        self.rgbd_info = RgbdCameraMetaInfo(self.camera_serial_number, self.link_to_joint_id_map)

    def BuildJointNameInfo(self):
        ''' Peruses all joints in Pybullet, and ...
            - Assembles a map from their joint name to their joint index
            - Assembles a map from their joint name to their parent link name
        '''
        num_joints = pybullet.getNumJoints(self.kuka_id)
        self.iiwa_wsg_joint_name_to_id = {}
        self.link_to_joint_id_map = {}
        for i in range(num_joints):
          joint_info = pybullet.getJointInfo(self.kuka_id, i)
          self.iiwa_wsg_joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]
          self.link_to_joint_id_map[joint_info[12].decode("UTF-8")] = joint_info[0]

    def BuildMotorIdList(self):
        self.iiwa_motor_id_list = [
            self.iiwa_wsg_joint_name_to_id[motor_name] for motor_name in IIWA_CONTROLLED_JOINTS
        ]
        self.schunk_motor_id_list = [
            self.iiwa_wsg_joint_name_to_id[motor_name] for motor_name in SCHUNK_CONTROLLED_JOINTS
        ]

    def HandleIiwaCommand(self, channel, data):
        self.iiwa_command_lock.acquire()
        try:
            msg = lcmt_iiwa_command.decode(data)
            for i in range(msg.num_joints):
                self.last_iiwa_position_command[i] = msg.joint_position[i]
        except Exception as e:
            print "Exception ", e, " in lcm iiwa command handler"
        self.iiwa_command_lock.release()

    def GetIiwaPositionCommand(self):
        self.iiwa_command_lock.acquire()
        command = deepcopy(self.last_iiwa_position_command)
        self.iiwa_command_lock.release()
        return command

    def PublishIiwaStatus(self):
        status_msg = lcmt_iiwa_status()
        status_msg.utime = time.time() * 1E6
        status_msg.num_joints = len(IIWA_CONTROLLED_JOINTS)
        # Get joint state info
        states = pybullet.getJointStates(self.kuka_id, self.iiwa_motor_id_list)
        positions = [states[i][0] for i in range(status_msg.num_joints)]
        velocities = [states[i][1] for i in range(status_msg.num_joints)]
        torques = [states[i][3] for i in range(status_msg.num_joints)]
        status_msg.joint_position_measured = positions
        status_msg.joint_velocity_estimated = velocities
        status_msg.joint_position_ipo = [0]*status_msg.num_joints
        status_msg.joint_torque_measured = torques
        status_msg.joint_torque_commanded = [0.]*status_msg.num_joints
        status_msg.joint_torque_external = [0.]*status_msg.num_joints

        self.iiwa_command_lock.acquire()
        status_msg.joint_position_commanded = deepcopy(self.last_iiwa_position_command)
        self.iiwa_command_lock.release()

        self.lc.publish("IIWA_STATUS", status_msg.encode())


    def HandleSchunkCommand(self, msg):
        self.schunk_command_lock.acquire()
        try:
            self.last_schunk_position_command = [-msg.position_mm*0.005, msg.position_mm*0.005]
            self.last_schunk_torque_command = [msg.force*10, msg.force*10]
        except Exception as e:
            print "Exception ", e, " in ros schunk command handler"
        self.schunk_command_lock.release()

    def GetSchunkCommand(self):
        self.schunk_command_lock.acquire()
        position_command = deepcopy(self.last_schunk_position_command)
        torque_command = deepcopy(self.last_schunk_torque_command)
        self.schunk_command_lock.release()
        return position_command, torque_command

    def PublishSchunkStatus(self):
        # Get joint state info
        states = pybullet.getJointStates(self.kuka_id, self.schunk_motor_id_list)

        msg = wsg50_msgs.msg.WSG_50_state()
        msg.header.stamp = rospy.Time.now()
        # Distance *between* paddles
        msg.position_mm = (-states[0][0] + states[1][0])*1000.
        msg.force = (-states[0][3] + states[1][3])/2.
        # Rate of change of distance between paddles
        msg.speed_mm_per_s = (-states[0][1] + states[1][1])*1000.
        self.schunk_status_publisher.publish(msg)

    def DoDepthRendering(self):
        # Get the state of the camera link
        linkState = pybullet.getLinkState(self.kuka_id, 
            self.rgbd_info.depth_extrinsics["parent_joint_id"],
            computeLinkVelocity=0)
        
        # Compute the camera's eye position
        cameraWorldPosition, cameraWorldOrientation = \
            pybullet.multiplyTransforms(
                linkState[0], linkState[1],
                self.rgbd_info.depth_extrinsics["pose_xyz"], 
                self.rgbd_info.depth_extrinsics["pose_quat"])

        # Use that to form Forward and Up vectors for the camera
        cameraRotationMatrix = np.reshape(np.array(pybullet.getMatrixFromQuaternion(cameraWorldOrientation)), [3, 3])
        cameraForward = cameraRotationMatrix.dot(np.array(CAMERA_FORWARD_VEC))
        cameraUp = cameraRotationMatrix.dot(np.array(CAMERA_UP_VEC))

        # Assemble a view matrix
        viewMatrix = pybullet.computeViewMatrix(cameraWorldPosition, cameraWorldPosition+cameraForward, cameraUp)

        # Compute the FOV and aspect from the camera intrinsics
        camera_width = self.rgbd_info.depth_info_msg.width
        camera_height = self.rgbd_info.depth_info_msg.height
        camera_fov_x = 2 * math.atan2(camera_height, 2 * self.rgbd_info.depth_info_msg.P[0]) * 180. / math.pi
        camera_fov_y = 2 * math.atan2(camera_width, 2 * self.rgbd_info.depth_info_msg.P[5]) * 180. / math.pi
        camera_aspect = float(camera_width) / float(camera_height)
        # Use the computed FOV to produce a projection matrix
        # (Note: I'm sure you can directly compute a projection Matrix
        # from the intrinsics, but I'm not sure what the mapping is --
        # OpenCV makes a lot of assumptions about their projection matrix
        # details and scaling. It's not exactly a camera intrinsic matrix...
        # So this is easier.)
        min_range = self.rgbd_info.depth_min_range
        max_range = self.rgbd_info.depth_max_range
        projectionMatrix = pybullet.computeProjectionMatrixFOV(camera_fov_x, camera_aspect,min_range, max_range)

        images = pybullet.getCameraImage(camera_width, camera_height, viewMatrix, projectionMatrix, shadow=0,lightDirection=[1,1,1],renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

        # Rescale depth buffer from [0, 1] to real depth following
        # formula from https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer
        gtDepthImage = (max_range * min_range) / (max_range + images[3] * (min_range - max_range))

        # Calculate normals before adding noise
        if self.rgbd_normal_limit > 0.:
            gtNormalImage = np.absolute(cv2.Scharr(gtDepthImage, cv2.CV_32F, 1, 0)) +  np.absolute(cv2.Scharr(gtDepthImage, cv2.CV_32F, 0, 1))
            _, normalThresh = cv2.threshold(gtNormalImage, self.rgbd_normal_limit, 1., cv2.THRESH_BINARY_INV)

        if self.rgbd_noise > 0.0:
            noiseMat = np.random.randn(camera_height, camera_width)*self.rgbd_noise
            gtDepthImage += noiseMat

        depthImage = gtDepthImage

        if self.rgbd_projector_baseline > 0.0:
            K = np.reshape(np.array(self.rgbd_info.depth_info_msg.K), [3, 3])

            # How much does each depth point project laterally (in the axis of the camera-projector pair?)
            x_indices_im = np.tile(np.arange(camera_width), [camera_height,1])
            x_projection = (x_indices_im - K[0, 2]) * gtDepthImage / K[0, 0]

            # For a fixed shift...
            for shift_amt in range(-50, -5, 5):
                imshift_tf_matrix = np.array([[1., 0., shift_amt], [0., 1., 0.]])
                shifted_x_projection = cv2.warpAffine(x_projection, imshift_tf_matrix, (camera_width, camera_height), borderMode=cv2.BORDER_REPLICATE)
                shifted_gt_depth = cv2.warpAffine(gtDepthImage, imshift_tf_matrix, (camera_width, camera_height), borderMode=cv2.BORDER_CONSTANT, borderValue=float(np.max(gtDepthImage)))

                # (projected test point - projected original point) dot producted with vector perpendicular to sample point and projector origin
                error_im = (shifted_x_projection - x_projection)*(-gtDepthImage) + \
                       (shifted_gt_depth - gtDepthImage)*(x_projection - self.rgbd_projector_baseline)
                _, error_thresh = cv2.threshold(error_im, 0., 1., cv2.THRESH_BINARY_INV)
                depthImage = depthImage * error_thresh

        # Apply normal limiting
        if self.rgbd_normal_limit > 0.:
            depthImage *= normalThresh

        #im = cvUtils.generateGridOfImages(
        #    [
        #        cvUtils.generateColorMap(gtDepthImage),
        #        cvUtils.generateColorMap(gtNormalImage, -0.1, .1),
        #        cvUtils.generateColorMap(normalThresh),
        #        cvUtils.generateColorMap(depthImage)
        #    ], 3, 10)
        #cv2.imshow("depthcorruption", im)
        #cv2.waitKey(1)

        # Convert and publish!        
        depthMsg = self.cv_bridge.cv2_to_imgmsg((depthImage*1000).astype('uint16'), "passthrough")

        # Construct a header with the current timestamp and the RGB
        # frame, and publish RGB camera info and the RGB camera image
        now_header = std_msgs.msg.Header()
        now_header.stamp = rospy.Time.now()
        now_header.frame_id = self.rgbd_info.frames['DEPTH_FRAME']
        self.rgbd_info.depth_info_msg.header = now_header
        self.depth_info_publisher.publish(self.rgbd_info.depth_info_msg)
        depthMsg.header = now_header
        self.depth_publisher.publish(depthMsg)

    def DoRgbRendering(self):
        # Get the state of the camera link
        linkState = pybullet.getLinkState(self.kuka_id, 
            self.rgbd_info.rgb_extrinsics["parent_joint_id"],
            computeLinkVelocity=0)
        
        # Compute the camera's eye position
        cameraWorldPosition, cameraWorldOrientation = \
            pybullet.multiplyTransforms(
                linkState[0], linkState[1],
                self.rgbd_info.rgb_extrinsics["pose_xyz"], 
                self.rgbd_info.rgb_extrinsics["pose_quat"])

        # Use that to form Forward and Up vectors for the camera
        cameraRotationMatrix = np.reshape(np.array(pybullet.getMatrixFromQuaternion(cameraWorldOrientation)), [3, 3])
        cameraForward = cameraRotationMatrix.dot(np.array(CAMERA_FORWARD_VEC))
        cameraUp = cameraRotationMatrix.dot(np.array(CAMERA_UP_VEC))

        # Assemble a view matrix
        viewMatrix = pybullet.computeViewMatrix(cameraWorldPosition, cameraWorldPosition+cameraForward, cameraUp)

        # Compute the FOV and aspect from the camera intrinsics
        camera_width = self.rgbd_info.rgb_info_msg.width
        camera_height = self.rgbd_info.rgb_info_msg.height
        camera_fov_x = 2 * math.atan2(camera_height, 2 * self.rgbd_info.rgb_info_msg.P[0]) * 180. / math.pi
        camera_fov_y = 2 * math.atan2(camera_width, 2 * self.rgbd_info.rgb_info_msg.P[5]) * 180. / math.pi
        camera_aspect = float(camera_width) / float(camera_height)
        # Use the computed FOV to produce a projection matrix
        # (Note: I'm sure you can directly compute a projection Matrix
        # from the intrinsics, but I'm not sure what the mapping is --
        # OpenCV makes a lot of assumptions about their projection matrix
        # details and scaling. It's not exactly a camera intrinsic matrix...
        # So this is easier.)
        projectionMatrix = pybullet.computeProjectionMatrixFOV(camera_fov_x, camera_aspect, IIWA_RGB_CAMERA_MIN_DISTANCE, IIWA_RGB_CAMERA_MAX_DISTANCE)

        images = pybullet.getCameraImage(camera_width, camera_height, viewMatrix, projectionMatrix, shadow=0,lightDirection=[1,1,1],renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

        rgbImage = images[2]
        # Convert and publish!        
        rgbMsg = self.cv_bridge.cv2_to_imgmsg(cv2.cvtColor(rgbImage, cv2.COLOR_BGRA2BGR), "rgb8")

        # Construct a header with the current timestamp and the RGB
        # frame, and publish RGB camera info and the RGB camera image
        now_header = std_msgs.msg.Header()
        now_header.stamp = rospy.Time.now()
        now_header.frame_id = self.rgbd_info.frames['RGB_FRAME']
        self.rgbd_info.rgb_info_msg.header = now_header
        self.rgb_info_publisher.publish(self.rgbd_info.rgb_info_msg)
        rgbMsg.header = now_header
        self.rgb_publisher.publish(rgbMsg)


    def UpdateRtv(self):
        for i, object_id in enumerate(self.object_ids):
            pos, quat = pybullet.getBasePositionAndOrientation(object_id)
            # this pos is in COM frame... offset to Drake's preference
            # of URDF link frame.
            # Should only be a problem for *root* links,
            # (i.e. index 1 in Drake), as the rest are relative
            # to that.
            com_root = np.array(self.rbts[i].get_body(1).get_center_of_mass())
            # Ugh conve
            pos = np.array(pos) - com_root
            rpy = np.array(pybullet.getEulerFromQuaternion(quat))
            state_vec = np.hstack([pos, rpy])

            N = pybullet.getNumJoints(object_id)
            if N > 0:
                jointInfo = pybullet.getJointStates(object_id, range(pybullet.getNumJoints(object_id)))
                print "Warning, untested. Remove this print if this works..."
                state_vec = np.hstack([state_vec, np.array([j[0] for j in jointInfo])])
            self.rtv.publishRigidBodyTree(self.rbts[i], state_vec, [1., 1., 1., 1.], ["object", str(i)], True)

    def RunSim(self):
        # Run simulation with time control
        start_time = time.time()
        sim_time = 0.0
        avg_sim_rate = -1.
        sim_rate_RC = 0.01 # RC time constant for estimating sim rate
        sim_rate_alpha = self.timestep / (sim_rate_RC + self.timestep) 
        last_print_time = time.time() - 100

        last_status_send = 0
        last_render = 0

        keep_going = True
        while 1:
            start_step_time = time.time()

            # Resolve new commands for the IIWA
            self.lc.handle_timeout(1)
            pybullet.setJointMotorControlArray(
                self.kuka_id,
                self.iiwa_motor_id_list,
                controlMode=pybullet.POSITION_CONTROL,
                targetPositions=self.GetIiwaPositionCommand(),
                forces=IIWA_MAX_JOINT_FORCES
                )

            # Resolve new commands for the gripper
            # ROS spinonce?
            schunk_position_command, schunk_torque_command = self.GetSchunkCommand()
            pybullet.setJointMotorControlArray(
                self.kuka_id,
                self.schunk_motor_id_list,
                controlMode=pybullet.POSITION_CONTROL,
                targetPositions=schunk_position_command,
                forces=schunk_torque_command,
                positionGains=[1000., 1000.]
                )

            # Do a sim step
            pybullet.stepSimulation()
            sim_time += self.timestep

            # Publish state
            if (sim_time - last_status_send) > 0.033:
                self.PublishIiwaStatus()
                self.PublishSchunkStatus()
                last_status_send = sim_time
                self.UpdateRtv()

            # Render
            if (sim_time - last_render) > 0.333:
                self.DoDepthRendering()
                self.DoRgbRendering()
                last_render = sim_time

            events = pybullet.getKeyboardEvents()
            for key in events.keys():
                if events[key] & pybullet.KEY_WAS_TRIGGERED:
                    if key == ord('q'):
                        return False
                    elif key == ord('r'):
                        print("Restarting")
                        return True

            end_step_time = time.time()

            # THINGS BELOW HERE MUST HAPPEN QUICKLY FOR TIME ESTIMATION TO BE ACCURATE

            # Estimate sim rate
            this_sim_step_rate = self.timestep / (end_step_time - start_step_time)
            if avg_sim_rate < 0:
                # Initialization case
                avg_sim_rate = this_sim_step_rate
            else:
                avg_sim_rate = this_sim_step_rate * sim_rate_alpha + (1. - sim_rate_alpha) * avg_sim_rate

            # Sleep if we're running too fast
            elapsed = end_step_time - start_time
            target_sim_time = elapsed * self.rate
            if sim_time > target_sim_time:
                time.sleep(sim_time - target_sim_time)

            if time.time() - last_print_time > 0.1:
                last_print_time = time.time()
                print("Overall sim rate: ", sim_time / target_sim_time, ", current sim rate: ", avg_sim_rate, " at time ", sim_time)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Configuration file to simulate.", type=str)
    parser.add_argument("-r", "--rate", help="Desired simulation rate (fraction of realtime)", type=float, default=1.0)
    parser.add_argument("-t", "--timestep", help="Simulation timestep", type=float, default=0.001)
    parser.add_argument("--rgbd_noise", help="Normal noise injected to RGBD depth returns", type=float, default=0.001)
    parser.add_argument("--rgbd_projector_baseline", help="RGBD projector baseline used to calculate depth shadowing", type=float, default=0.1)
    parser.add_argument("--rgbd_normal_limit", help="Threshold for rejecting high-normal depth returns. (Smaller is more stringent, 0.0 to turn off.)", type=float, default=0.05)
    parser.add_argument("--headless", help="Run without GUI.", action="store_true")
    parser.add_argument("--camera_serial_number", type=str, default="carmine_1", )
    args = parser.parse_args()

    rospy.init_node('pybullet_iiwa_rlg_simulation', anonymous=True)

    #cv2.namedWindow("depthcorruption")

    # Set up a simulation with a ground plane and desired timestep
    if args.headless:
        physicsClient = pybullet.connect(pybullet.DIRECT)
    else:
        physicsClient = pybullet.connect(pybullet.GUI)
    
    sim = IiwaRlgSimulator(args.config, args.timestep, args.rate,
            rgbd_projector_baseline = args.rgbd_projector_baseline,
            rgbd_noise = args.rgbd_noise,
            rgbd_normal_limit = args.rgbd_normal_limit,
            camera_serial_number=args.camera_serial_number)

    keep_simulating = True
    while keep_simulating:
        sim.ResetSimulation()
        keep_simulating = sim.RunSim()

