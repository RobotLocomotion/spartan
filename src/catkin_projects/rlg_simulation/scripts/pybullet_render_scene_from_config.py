#!/usr/bin/env python

# Renders a scene from a given perspective. Uses calibration file
# to figure out tf between the rgb and depth camera and properly
# renders them from different perspectives.
#
# Arguments:
#   config: Configuration file that describes the scene. See ../config
#           for examples
#   camera: Camera serial number, to look up its calibration parameters.
#   origin_x: rgb camera origin
#   origin_y:
#   origin_z:
#   forward_x: rgb camera forward dir
#   forward_y:
#   forward_z:
#   up_x: rgb camera up direction
#   up_y:
#   up_z:
#   output_prefix: Prefix for output files. Appends _rgb.jpg, _depth.jpg.

import argparse
import cv2
import os
import math
import numpy as np
import pybullet as p
import time
import yaml

import pybullet
from pybullet_iiwa_rlg_simulation import IiwaRlgSimulator

# TODO pull from calibration file
IIWA_DEPTH_CAMERA_MIN_DISTANCE = 0.35 
IIWA_DEPTH_CAMERA_MAX_DISTANCE = 3.0
IIWA_RGB_CAMERA_MIN_DISTANCE = 0.1
IIWA_RGB_CAMERA_MAX_DISTANCE = 10.

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Configuration file to render.", type=str)
    parser.add_argument("origin_x", help="Rgb camera origin, x", type=float)
    parser.add_argument("origin_y", help="Rgb camera origin, y", type=float)
    parser.add_argument("origin_z", help="Rgb camera origin, z", type=float)
    parser.add_argument("forward_x", help="Rgb camera forward direction, x", type=float)
    parser.add_argument("forward_y", help="Rgb camera forward direction, y", type=float)
    parser.add_argument("forward_z", help="Rgb camera forward direction, z", type=float)
    parser.add_argument("up_x", help="Rgb camera up direction, x", type=float)
    parser.add_argument("up_y", help="Rgb camera up direction, y", type=float)
    parser.add_argument("up_z", help="Rgb camera up direction, z", type=float)
    parser.add_argument("--camera_serial_number", help="Camera calibration file", type=str, default="carmine_1")
    parser.add_argument("-o", "--prefix", help="Camera calibration file", type=str, default="render")
    parser.add_argument("--rgbd_noise", help="Normal noise injected to RGBD depth returns", type=float, default=0.001)
    parser.add_argument("--rgbd_projector_baseline", help="RGBD projector baseline used to calculate depth shadowing", type=float, default=0.1)
    parser.add_argument("--rgbd_normal_limit", help="Threshold for rejecting high-normal depth returns. (Smaller is more stringent, 0.0 to turn off.)", type=float, default=0.05)
    args = parser.parse_args()

    physicsClient = pybullet.connect(pybullet.DIRECT)
    
    # Rate and timestep are irrelevant
    sim = IiwaRlgSimulator(args.config, 0.01, 1.0,
            rgbd_projector_baseline = args.rgbd_projector_baseline,
            rgbd_noise = args.rgbd_noise,
            rgbd_normal_limit = args.rgbd_normal_limit,
            camera_serial_number=args.camera_serial_number)

    sim.ResetSimulation()


    # Do rendering manually, but steal the RGB->Depth frame offset from
    # the camera calib

    linkState = pybullet.getLinkState(sim.kuka_id, 
        sim.rgbd_info.rgb_extrinsics["parent_joint_id"],
        computeLinkVelocity=0)
    depthCameraWorldPosition, depthCameraWorldOrientation = \
            pybullet.multiplyTransforms(
                linkState[0], linkState[1],
                sim.rgbd_info.depth_extrinsics["pose_xyz"], 
                sim.rgbd_info.depth_extrinsics["pose_quat"])
    
    linkState = pybullet.getLinkState(sim.kuka_id, 
        sim.rgbd_info.depth_extrinsics["parent_joint_id"],
        computeLinkVelocity=0)    
    rgbCameraWorldPosition, rgbCameraWorldOrientation = \
            pybullet.multiplyTransforms(
                linkState[0], linkState[1],
                sim.rgbd_info.rgb_extrinsics["pose_xyz"], 
                sim.rgbd_info.rgb_extrinsics["pose_quat"])
    rgbCameraWorldPositionInv, rgbCameraWorldOrientationInv = \
            pybullet.invertTransform(rgbCameraWorldPosition,
                                     rgbCameraWorldOrientation)


    rgbToDepthOffsetPosition, rgbToDepthOffsetOrientation = \
            pybullet.multiplyTransforms(
                rgbCameraWorldPositionInv, rgbCameraWorldOrientationInv,
                depthCameraWorldPosition, 
                depthCameraWorldOrientation)


    ####  RGB
    # Assemble a view matrix
    origin = np.array([args.origin_x, args.origin_y, args.origin_z])
    forward = np.array([args.forward_x, args.forward_y, args.forward_z])
    up = np.array([args.up_x, args.up_y, args.up_z])


    viewMatrix = pybullet.computeViewMatrix(origin, origin+forward, up)

    # Compute the RGB FOV and aspect from the camera intrinsics
    camera_width = sim.rgbd_info.rgb_info_msg.width
    camera_height = sim.rgbd_info.rgb_info_msg.height
    camera_fov_x = 2 * math.atan2(camera_height, 2 * sim.rgbd_info.rgb_info_msg.P[0]) * 180. / math.pi
    camera_fov_y = 2 * math.atan2(camera_width, 2 * sim.rgbd_info.rgb_info_msg.P[5]) * 180. / math.pi
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

    #### Depth
    offsetRotationMatrix = np.reshape(np.array(pybullet.getMatrixFromQuaternion(rgbToDepthOffsetOrientation)), [3, 3])

    origin += offsetRotationMatrix.dot(rgbToDepthOffsetPosition) 
    forward = offsetRotationMatrix.dot(forward)
    up = offsetRotationMatrix.dot(up)

    viewMatrix = pybullet.computeViewMatrix(origin, origin+forward, up)

    # Compute the RGB FOV and aspect from the camera intrinsics
    camera_width = sim.rgbd_info.depth_info_msg.width
    camera_height = sim.rgbd_info.depth_info_msg.height
    camera_fov_x = 2 * math.atan2(camera_height, 2 * sim.rgbd_info.depth_info_msg.P[0]) * 180. / math.pi
    camera_fov_y = 2 * math.atan2(camera_width, 2 * sim.rgbd_info.depth_info_msg.P[5]) * 180. / math.pi
    camera_aspect = float(camera_width) / float(camera_height)
    projectionMatrix = pybullet.computeProjectionMatrixFOV(camera_fov_x, camera_aspect, IIWA_DEPTH_CAMERA_MIN_DISTANCE, IIWA_DEPTH_CAMERA_MAX_DISTANCE)

    images = pybullet.getCameraImage(camera_width, camera_height, viewMatrix, projectionMatrix, shadow=0,lightDirection=[1,1,1],renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

    # Rescale depth buffer from [0, 1] to real depth following
    # formula from https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer
    min_range = sim.rgbd_info.depth_min_range
    max_range = sim.rgbd_info.depth_max_range
    gtDepthImage = (max_range * min_range) / (max_range + images[3] * (min_range - max_range))

    # Calculate normals before adding noise
    if sim.rgbd_normal_limit > 0.:
        gtNormalImage = np.absolute(cv2.Scharr(gtDepthImage, cv2.CV_32F, 1, 0)) +  np.absolute(cv2.Scharr(gtDepthImage, cv2.CV_32F, 0, 1))
        _, normalThresh = cv2.threshold(gtNormalImage, sim.rgbd_normal_limit, 1., cv2.THRESH_BINARY_INV)

    if sim.rgbd_noise > 0.0:
        noiseMat = np.random.randn(camera_height, camera_width)*sim.rgbd_noise
        gtDepthImage += noiseMat

    depthImage = gtDepthImage

    if sim.rgbd_projector_baseline > 0.0:
        K = np.reshape(np.array(sim.rgbd_info.depth_info_msg.K), [3, 3])

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
                   (shifted_gt_depth - gtDepthImage)*(x_projection - sim.rgbd_projector_baseline)
            _, error_thresh = cv2.threshold(error_im, 0., 1., cv2.THRESH_BINARY_INV)
            depthImage = depthImage * error_thresh

    # Apply normal limiting
    if sim.rgbd_normal_limit > 0.:
        depthImage *= normalThresh

    rgbImage, depthImage
    cv2.imwrite(args.prefix + "_rgb.jpg", rgbImage)
    cv2.imwrite(args.prefix + "_depth.jpg", depthImage)
