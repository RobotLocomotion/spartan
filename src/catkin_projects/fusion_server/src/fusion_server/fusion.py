#!/usr/bin/env python

#system
import rospy
import os
import sys
import time
import subprocess
import copy
import numpy as np

# ply reader
from plyfile import PlyData, PlyElement

# ROS
import tf2_ros

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils

# ros srv
import fusion_server.srv
from fusion_server.srv import *
from fusion_server.numpy_pc2 import array_to_xyz_pointcloud2f



# this function taken from here:
# https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)


class TFWrapper(object):

    def __init__(self):
        self.tfBuffer = None
        self.tfListener = None
        self.setup()

    def setup(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def getBuffer(self):
        while self.tfBuffer is None:
            time.sleep(0.1)

        return self.tfBuffer



class FusionServer(object): 

    def __init__(self, camera_serial_number="carmine_1"):
        self.camera_serial_number = camera_serial_number
        self.bagging = False
        self.rosbag_proc = None
        self.tfBuffer = None
        storedPosesFile = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects', 'station_config','RLG_iiwa_1','stored_poses.yaml')
        self.storedPoses = spartanUtils.getDictFromYamlFilename(storedPosesFile)
        self.robotService = rosUtils.RobotService(self.storedPoses['header']['joint_names'])
        self.setupConfig()
        self.setupSubscribers()
        self.setupPublishers()
        self.setupTF()
        self.setupCache()

    def setupConfig(self):
        self.config = dict()
        self.config['scan'] = dict()
        #self.config['scan']['pose_list'] = ['scan_back', 'scan_left', 'scan_top', 'scan_right', 'scan_back']
        # with the Grasping group
        #self.config['scan']['pose_group'] = 'Grasping'
        #self.config['scan']['pose_list'] = ['scan_left_close', 'scan_left', 'scan_left_center', 'scan_above_table_far', 'scan_right_center', 'scan_right', 'scan_right_close']


        self.config['scan']['pose_group'] = 'Elastic Fusion'
        self.config['scan']['pose_list'] = ['home', 'home_closer', 'center_right', 'right', 'right_low', 'right_low_closer', 'center_right', 'home_closer', 'center_left_closer', 'center_left_low_closer', 'left_low', 'left_mid', 'center_left_low', 'center_left_low_closer', 'center_left_closer', 'home_closer', 'top_down', 'top_down_right', 'top_down_left']
        self.config['scan']['pose_list_quick'] = ['home_closer', 'top_down', 'top_down_right', 'top_down_left', 'home']

        self.config['speed'] = dict()
        self.config['speed']['scan'] = 15
        self.config['speed']['fast'] = 30

        self.config['spin_rate'] = 1

        
        self.config['home_pose_name'] = 'home'
        self.config['sleep_time_before_bagging'] = 2.0
        self.config['world_frame'] = 'base'
        self.config['sleep_time_at_each_pose'] = 0.5

        self.config["reconstruction_frame_id"] = "fusion_reconstruction"

        self.topics_to_bag = [
            "/tf",
            "/tf_static",
            "/camera_"+self.camera_serial_number+"/depth_registered/sw_registered/image_rect",
            "/camera_"+self.camera_serial_number+"/depth_registered/sw_registered/camera_info",
            "/camera_"+self.camera_serial_number+"/rgb/camera_info",
            "/camera_"+self.camera_serial_number+"/rgb/image_rect_color"
        ]

        self.topics_dict = dict()
        self.topics_dict['rgb'] = "/camera_"+self.camera_serial_number+"/rgb/image_rect_color"
        self.topics_dict['depth'] = "/camera_"+self.camera_serial_number+"/depth_registered/sw_registered/image_rect"
        self.topics_dict['camera_info'] = "/camera_"+self.camera_serial_number+"/rgb/camera_info"

    def setupTF(self):
        tfWrapper = TFWrapper()
        tfBuffer = tfWrapper.getBuffer()
        if self.tfBuffer is None:
            self.tfBuffer = tfBuffer

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def setupSubscribers(self):
        self.subscribers = dict()

        self.subscribers['rgb'] = rosUtils.SimpleSubscriber(self.topics_dict['rgb'], sensor_msgs.msg.Image)

        self.subscribers['depth'] = rosUtils.SimpleSubscriber(self.topics_dict['depth'], sensor_msgs.msg.Image)

    def setupPublishers(self):
        topic_name = "fusion_reconstruction"
        self.pointcloud_publisher = rospy.Publisher(topic_name, sensor_msgs.msg.PointCloud2, queue_size=1)

    def startImageSubscribers(self):
        for key, sub in self.subscribers.iteritems():
            sub.start()

    def stopImageSubscribers(self):
        for key, sub in self.subscribers.iteritems():
            sub.stop()

    def setupCache(self):
        self.cache = dict()

    def flushCache(self):
        self.setupCache()

    """
    Get transform from rgb_optical_frame to world
    """
    def getRGBOpticalFrameToWorldTransform(self):
        rgbOpticalFrameToWorld = self.tfBuffer.lookup_transform(self.config['world_frame'], rosUtils.getRGBOpticalFrameName(self.camera_serial_number), rospy.Time(0))
        return rgbOpticalFrameToWorld

    def start_bagging(self):
        self.flushCache()

        bagfile_name = "fusion" + str(time.time())
        bagfile_directory = os.path.join(spartanUtils.getSpartanSourceDir(), 'sandbox', 'fusion', bagfile_name)
        

        # make bagfile directory with name
        os.system("mkdir -p " + bagfile_directory)

        topics_to_bag = [
            "/tf",
            "/tf_static",
            "/camera_"+self.camera_serial_number+"/depth_registered/sw_registered/image_rect",
            "/camera_"+self.camera_serial_number+"/depth_registered/sw_registered/camera_info",
            "/camera_"+self.camera_serial_number+"/rgb/camera_info",
            "/camera_"+self.camera_serial_number+"/rgb/image_rect_color"
        ]

        # add simple subscribers to fix xtion driver issues
        self.startImageSubscribers()

        rospy.loginfo("started image subscribers, sleeping for %d seconds", self.config['sleep_time_before_bagging'])
        # sleep for two seconds to allow for xtion driver compression issues to be resolved
        
        # rospy.sleep(self.config['sleep_time_before_bagging'])


        # get camera to world transform
        self.cache['point_cloud_to_world_stamped'] = self.getRGBOpticalFrameToWorldTransform()
        transform_stamped = self.cache['point_cloud_to_world_stamped']
        
        # build up command string
        rosbag_cmd = "rosbag record"
        rosbag_cmd += " -O " + bagfile_name
        for i in topics_to_bag:
            rosbag_cmd += " " + i

        # add some visibility
        print rosbag_cmd

        # start bagging
        rosbag_proc = subprocess.Popen(rosbag_cmd, stdin=subprocess.PIPE, shell=True, cwd=bagfile_directory)

        rospy.loginfo("sleeping while waiting for bagging to start")
        rospy.sleep(2.0)
        return os.path.join(bagfile_directory, bagfile_name+".bag"), rosbag_proc

    def handle_start_bagging_fusion_data(self, req):
        ## check if bagging already
        if self.bagging:
            return StartBaggingFusionDataResponse("ERROR: Already bagging!")

        ## start bagging
        filepath, self.rosbag_proc = self.start_bagging()
        self.bagging = True

        ## return the full path string to the data
        print "Returning filepath"
        return StartBaggingFusionDataResponse(filepath)

    def handle_stop_bagging_fusion_data(self, req):
        ## check if bagging already
        if not self.bagging:
            return StopBaggingFusionDataResponse("ERROR: Not currently bagging! Nothing to stop...")

        ## stop bagging 
        terminate_ros_node("/record")                            # this is heavier weight but will not create a .active
        # self.rosbag_proc.send_signal(subprocess.signal.SIGINT) # this is a more direct way of stopping the rosbag, but will terminate it with a .active
        self.bagging = False
        self.stopImageSubscribers()

        return StopBaggingFusionDataResponse("success")

    def handle_perform_elastic_fusion(self, req):
        ## call executable for filename
        cmd = ". /opt/ros/kinetic/setup.sh && $SPARTAN_SOURCE_DIR/src/ElasticFusion/GUI/build/ElasticFusion -q -t 900 -d 2 -l " + req.bag_filepath

        cl_args = ""
        cl_args += " --ros_bag_filename " + req.bag_filepath
        cl_args += " --ros_image_depth_topic " + self.topics_dict['depth']
        cl_args += " --ros_image_rgb_topic " + self.topics_dict['rgb']
        cl_args += " --ros_cam_info_topic " + self.topics_dict['camera_info']

        cmd += cl_args

        rospy.loginfo("elastic fusion cmd = %s", cmd)

        os.system(cmd)

        ply_filename = req.bag_filepath + ".ply"

        # of type sensor_msgs.msg.PointCloud2
        point_cloud = self.convert_ply_to_pointcloud2(PlyData.read(ply_filename))
        
        res = PerformElasticFusionResponse()
        res.elastic_fusion_output.pointcloud_filepath = ply_filename
        res.elastic_fusion_output.point_cloud = point_cloud
        res.elastic_fusion_output.point_cloud_to_world_stamped = self.cache['point_cloud_to_world_stamped']


        return res

    def convert_ply_to_pointcloud2(self, plydata):

        cloud_arr = np.zeros((len(plydata.elements[0].data), 3))
        for i in xrange(len(plydata.elements[0].data)):
            cloud_arr[i] = list(plydata.elements[0].data[i])[:3]

        return array_to_xyz_pointcloud2f(cloud_arr)

    def handle_capture_scene_and_fuse(self, req):
        # Start bagging with own srv call

        # first move home
        home_pose_joint_positions = self.storedPoses[self.config['scan']['pose_group']][self.config['home_pose_name']]
        print home_pose_joint_positions
        self.robotService.moveToJointPosition(home_pose_joint_positions, maxJointDegreesPerSecond=self.config['speed']['fast'])

        try:
            start_bagging_fusion_data = rospy.ServiceProxy('start_bagging_fusion_data', StartBaggingFusionData)
            resp1 = start_bagging_fusion_data()
            bag_filepath = resp1.bag_filepath
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Move robot around
        for poseName in self.config['scan']['pose_list_quick']:
            print "moving to", poseName
            joint_positions = self.storedPoses[self.config['scan']['pose_group']][poseName]
            self.robotService.moveToJointPosition(joint_positions, maxJointDegreesPerSecond=self.config['speed']['scan'])
            rospy.sleep(self.config['sleep_time_at_each_pose'])

        # Stop bagging with own srv call
        try:
            stop_bagging_fusion_data = rospy.ServiceProxy('stop_bagging_fusion_data', StopBaggingFusionData)
            resp2 = stop_bagging_fusion_data()
            print resp2.status, "stopped bagging"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # move home, now send stuff off to fusion server
        self.robotService.moveToJointPosition(home_pose_joint_positions, maxJointDegreesPerSecond=self.config['speed']['fast'])

        # Perform fusion

        try:
            perform_elastic_fusion = rospy.ServiceProxy('perform_elastic_fusion', PerformElasticFusion)
            resp3 = perform_elastic_fusion(resp1.bag_filepath)           
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # publish the pointcloud to RVIZ
        elastic_fusion_output = resp3.elastic_fusion_output
        self.cache['fusion_output'] = elastic_fusion_output
        self.publish_pointcloud_to_rviz(elastic_fusion_output.point_cloud, self.cache['point_cloud_to_world_stamped'])

        # extract all rgb and depth images, with timestamps
        path_to_extract_script = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects', 'fusion_server', 'scripts', 'extract_images_from_rosbag.py')
        destination_folder = os.path.join(os.path.dirname(resp1.bag_filepath), "images")
        os.system("mkdir -p " + destination_folder)
        cmd = "python " + path_to_extract_script + " " + resp1.bag_filepath + " " + destination_folder + " '/camera_carmine_1/rgb/image_rect_color' bgr8"
        print cmd
        os.system(cmd)

        return CaptureSceneAndFuseResponse(elastic_fusion_output)

    def run_fusion_data_server(self):
        s = rospy.Service('start_bagging_fusion_data', StartBaggingFusionData, self.handle_start_bagging_fusion_data)
        s = rospy.Service('stop_bagging_fusion_data', StopBaggingFusionData, self.handle_stop_bagging_fusion_data)
        s = rospy.Service('perform_elastic_fusion', PerformElasticFusion, self.handle_perform_elastic_fusion)
        s = rospy.Service('capture_scene_and_fuse', CaptureSceneAndFuse, self.handle_capture_scene_and_fuse)
        print "Ready to capture fusion data."

        rate = rospy.Rate(self.config['spin_rate'])
        while not rospy.is_shutdown():
            if 'fusion_output' in self.cache:
                fusion_output = self.cache['fusion_output']
                self.publish_pointcloud_to_rviz(fusion_output.point_cloud, self.cache['point_cloud_to_world_stamped'])

            # self.publish_reconstruction_to_world_transform()
            rate.sleep()


    def publish_pointcloud_to_rviz(self, point_cloud_2_msg, point_cloud_to_world_stamped):
        """
        Publishes out the reconstruction to rviz
        """

        rospy.loginfo("publishing pointcloud to rviz")
        header = point_cloud_2_msg.header
        header.stamp = rospy.Time.now()

        reconstruction_frame_id = self.config['reconstruction_frame_id']
        header.frame_id = reconstruction_frame_id

        self.pointcloud_publisher.publish(point_cloud_2_msg)
        self.publish_reconstruction_to_world_transform(point_cloud_to_world_stamped)


    def publish_reconstruction_to_world_transform(self, point_cloud_to_world_stamped=None):
        """
        Publish transform from reconstruction to world frame
        """
        if point_cloud_to_world_stamped is None:
            if 'point_cloud_to_world_stamped' in self.cache:
                point_cloud_to_world_stamped = self.cache['point_cloud_to_world_stamped']
            else:
                return
    

        reconstruction_frame_id = self.config['reconstruction_frame_id']

        reconstruction_to_world_stamped = copy.deepcopy(point_cloud_to_world_stamped)
        reconstruction_to_world_stamped.header.stamp = rospy.Time.now()
        reconstruction_to_world_stamped.child_frame_id = reconstruction_frame_id
        self.tf_broadcaster.sendTransform(reconstruction_to_world_stamped)