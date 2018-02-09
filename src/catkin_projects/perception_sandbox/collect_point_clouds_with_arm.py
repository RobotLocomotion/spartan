#!/usr/bin/env python
import sys
import os
import rospy
import numpy as np
import time

import sensor_msgs.msg
from spartan.utils.ros_utils import *
from spartan.utils.utils import poseFromTransform
from tf import TransformListener
import geometry_msgs
import robot_msgs.srv
import perception_sandbox.srv
import perception_sandbox.msg
import spartan_grasp_msgs
import sensor_msgs
import std_msgs.msg
import std_srvs.srv
from director import transformUtils

# Each goal is [camera loc][lookat pos][up dir]
goals = [
    [[0.2, -0.5, 1.2], [0.8, 0.0, 0.8], [0., 0., 1.0]],
    [[0.2, 0.0, 1.5], [0.8, 0.0, 0.8], [0., 0., 1.0]],
    [[0.2, 0.5, 1.2], [0.8, 0.0, 0.8], [0., 0., 1.0]]
    ]


class PointCloudObjectFittingServer(object):

    def __init__(self):
        # Subscribe to point clouds
        self.pc_subscriber = SimpleSubscriber("/camera_1112170110/depth_registered/points", sensor_msgs.msg.PointCloud2)
        self.pc_subscriber.start()
        self.base_frame_id = "base"
        self.camera_frame_id = "camera_1112170110_rgb_optical_frame"
        self.timeout = 5
        self.tf = TransformListener()

    def addPointCloud(self, minPt=[-10, -10., -10.], maxPt=[10., 10., 10.]):
        rospy.wait_for_service('/object_fitting/AddPointCloudAtPose', timeout=self.timeout)
        s = rospy.ServiceProxy('/object_fitting/AddPointCloudAtPose', perception_sandbox.srv.AddPointCloudAtPose)
        pcpose = spartan_grasp_msgs.msg.PointCloudWithTransform()

        while self.pc_subscriber.lastMsg is None:
            print "PC subscriber sees nothing! Waiting for one..."
            rospy.sleep(1)

        trans, quat = self.tf.lookupTransform(self.base_frame_id, self.camera_frame_id, rospy.Time(0))
        # USing this to get to tF matrix because the ordering of quaterion is weird
        tf_matrix = self.tf.fromTranslationRotation(trans, quat)

        pcpose.header.stamp = rospy.Time.now()
        pcpose.header.frame_id = "base"
        pcpose.point_cloud = self.pc_subscriber.lastMsg
        # What is with this horrible conversion chain...
        pcpose.point_cloud_to_base_transform.transform = ROSTransformMsgFromPose(poseFromTransform(transformUtils.getTransformFromNumpy(tf_matrix)))
        pcpose.camera_origin = listToPointMsg([0, 0, 0])
        response = s(pcpose, listToPointMsg(minPt), listToPointMsg(maxPt))
        return response

    def ResetPointClouds(self):
        rospy.wait_for_service('/object_fitting/ResetPointClouds', timeout=self.timeout)
        s = rospy.ServiceProxy('/object_fitting/ResetPointClouds', std_srvs.srv.Empty)
        response = s()
        return response

    def FitObjectsByIcp(self, meshes, scales, num_attempts = 10, initial_max_correspondence_distance = 0.05):
        rospy.wait_for_service('/object_fitting/FitObjectsByIcp', timeout=self.timeout)
        s = rospy.ServiceProxy('/object_fitting/FitObjectsByIcp', perception_sandbox.srv.FitObjectsByIcp)
        params = perception_sandbox.msg.IcpParams()
        params.num_attempts = num_attempts
        params.initial_max_correspondence_distance = initial_max_correspondence_distance
        params.max_iterations = 500
        strings = [std_msgs.msg.String(mesh) for mesh in meshes]
        response = s(strings, scales, params)
        return response



def cleanup_and_exit(code):
    sys.exit(code)

if __name__=="__main__":
    rospy.init_node("collect_point_clouds_with_arm", anonymous=True)

    robotService = RobotService.makeKukaRobotService()

    pcserver = PointCloudObjectFittingServer()
    pcserver.ResetPointClouds()

    for cameragoal in goals:
        hand_goal = np.array(cameragoal[0])
        look_goal = np.array(cameragoal[1])
        up_goal = np.array(cameragoal[2])
        tf = transformUtils.getLookAtTransform(look_goal, hand_goal, up_goal)

        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = ROSPoseMsgFromPose(poseFromTransform(tf))
        srvRequest = robot_msgs.srv.RunIKRequest()
        srvRequest.pose_stamped = poseStamped
        success = robotService.moveToCartesianPosition(srvRequest, timeout=5)
        if not success:
            print "robotService moveToCartesianPosition returned failure ", success
            cleanup_and_exit(1)

        rospy.sleep(3)
        rospy.sleep(3)
        pcserver.addPointCloud(minPt = [0.0, -0.5, -0.5], maxPt = [1.5, 0.5, 2.5])