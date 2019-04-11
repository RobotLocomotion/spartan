#!/usr/bin/env python

#system
import rospy
import os
import sys
import time
import subprocess
import copy
import numpy as np
import cv2
import shutil


# ply reader
from plyfile import PlyData, PlyElement

# ROS
import tf
import tf2_ros
import rospy
import rosbag
from cv_bridge import CvBridge


# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils

# ros srv
import fusion_server.srv
from fusion_server.srv import *
from fusion_server.numpy_pc2 import array_to_xyz_pointcloud2f
import fusion_server.tsdf_fusion as tsdf_fusion




ROS_BAGGING_NODE_NAME = "spartan_rosbag_node"



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



class ImageCapture(object):
    """
    Class used to capture synchronized images. It can also read them from a log
    """

    def __init__(self, rgb_topic, depth_topic, camera_info_topic,
        camera_frame, world_frame, rgb_encoding='bgr8'):

        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.tfBuffer = None
    

        self.rgb_encoding = rgb_encoding
        self.topics_dict = dict()
        self.topics_dict['rgb'] = rgb_topic
        self.topics_dict['depth'] = depth_topic
        self.camera_info_topic = camera_info_topic

        self.cv_bridge = CvBridge()

    def setupConfig(self):
        self.topics_dict = dict()
        self.topics_dict['rgb'] = "/camera_carmine_1/rgb/image_rect_color"
        self.topics_dict['depth'] = "/camera_carmine_1/depth_registered/sw_registered/image_rect"

        self.camera_info_topic = "/camera_carmine_1/rgb/camera_info"
        self.rgb_encoding = 'bgr8'

    def resetCache(self):
        self.depth_msgs = []
        self.rgb_msgs = []
        self.camera_to_base_transforms = []

    def setupTF(self):
        tfWrapper = TFWrapper()
        tfBuffer = tfWrapper.getBuffer()
        if self.tfBuffer is None:
            self.tfBuffer = tfBuffer

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def start(self):
        self.resetCache()
        self.subscribers['rgb'].start()
        self.subscribers['depth'].start()

    def getRGBOpticalFrameToWorldTransform(self, ros_time=None):
        if ros_time is None:
            ros_time = rospy.Time(0)
        rgbOpticalFrameToWorld = self.tfBuffer.lookup_transform(self.world_frame, self.camera_frame, ros_time)
        return rgbOpticalFrameToWorld

    def setupSubscribers(self):
        self.subscribers = {}
        self.subscribers['rgb'] = rosUtils.SimpleSubscriber(self.topics_dict['rgb'], sensor_msgs.msg.Image, externalCallback=self.onRgbImage)

        self.subscribers['depth'] = rosUtils.SimpleSubscriber(self.topics_dict['depth'], sensor_msgs.msg.Image, externalCallback=self.onDepthImage)

    def onRgbImage(self, msg):
        self.rgb_msgs.append(msg)

    def onDepthImage(self, msg):
        self.depth_msgs.append(msg)
        camera_to_world = self.getRGBOpticalFrameToWorldTransform(ros_time = msg.header.stamp)
        self.camera_to_base_transforms.append(camera_to_world)

    def getTransforms(self):
        for msg in self.depth_msgs:
            stamp = msg.header.stamp
            camera_to_world = self.getRGBOpticalFrameToWorldTransform(ros_time=stamp)
            self.camera_to_base_transforms.append(camera_to_world)

    def stop(self):
        self.subscribers['rgb'].stop()
        self.subscribers['depth'].stop()

    def synchronize_rgb_and_depth_msgs(self):
        self.rgb_timestamps = []
        for msg in self.rgb_msgs:
            stamp = msg.header.stamp
            print type(stamp)
            # self.rgb_timestamps.append()

    def load_ros_bag(self, ros_bag_filename):
        self.ros_bag = rosbag.Bag(ros_bag_filename, "r")

    def process_ros_bag(self, bag, output_dir, rgb_only=False):

        image_topics = []
        for key, topic in self.topics_dict.iteritems():
            image_topics.append(topic)


        # load all the images
        rgb_data = dict()
        rgb_data['msgs'] = []
        rgb_data['cv_img'] = []
        rgb_data['timestamps'] = []



        depth_data = dict()
        depth_data['msgs'] = []
        depth_data['cv_img'] = []
        depth_data['timestamps'] = []
        depth_data['camera_to_world'] = []

        print "image_topics: ", image_topics

        # extract TF information
        tf_t = rosUtils.setup_tf_transformer_from_ros_bag(bag, cache_time_secs=3600)

        log_rate = 100

        counter = 0
        for topic, msg, t in bag.read_messages(topics=image_topics):
            counter += 1

            # skip the first 30 images due to transform errors . . . 
            # if counter < 30:
            #     continue

            if counter % log_rate == 0:
                print "processing image message %d" %(counter)


            data = None
            if "rgb" in topic:
                cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding=self.rgb_encoding)
                data = rgb_data
            elif "depth" in topic:
                cv_img = rosUtils.depth_image_to_cv2_uint16(msg, bridge=self.cv_bridge)
                data = depth_data

                try:
                    # rot ix (x,y,z,w)
                    (trans, rot) = tf_t.lookupTransform(self.world_frame, self.camera_frame, msg.header.stamp)
                except:
                    print "wasn't able to get transform for image message %d, skipping" %(counter)
                    continue


                depth_data['camera_to_world'].append((trans, rot))
                

            # save the relevant data
            data['msgs'].append(msg)
            data['cv_img'].append(cv_img)
            data['timestamps'].append(msg.header.stamp.to_nsec())

        print "Extracted %d rgb images" %(len(rgb_data['msgs']))
        print "Extracted %d depth images" %(len(depth_data['msgs']))
        

        rgb_data['timestamps'] = np.array(rgb_data['timestamps'])

        # synchronize the images
        synchronized_rgb_imgs = []
        for idx, stamp in enumerate(depth_data['timestamps']):
            rgb_idx = ImageCapture.lookup_synchronized_image(stamp, rgb_data['timestamps'])
            synchronized_rgb_imgs.append(rgb_data['cv_img'][rgb_idx])
            if idx % log_rate == 0:
                print "depth image %d matched to rgb image %d" %(idx, rgb_idx)


        # save to a file
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)


        pose_data = dict()

        for idx, depth_img in enumerate(depth_data['cv_img']):
            rgb_img = synchronized_rgb_imgs[idx]

            rgb_filename = "%06i_%s.png" % (idx, "rgb")
            rgb_filename_full = os.path.join(output_dir, rgb_filename)

            depth_filename = "%06i_%s.png" % (idx, "depth")
            depth_filename_full = os.path.join(output_dir, depth_filename)

            if idx % log_rate == 0:
                print "writing image %d to file %s" %(idx, rgb_filename)
            
            cv2.imwrite(rgb_filename_full, rgb_img)
            if not rgb_only:
                cv2.imwrite(depth_filename_full, depth_img)

            pose_data[idx] = dict()
            d = pose_data[idx] 
            trans, rot = depth_data['camera_to_world'][idx]
            quat_wxyz = [rot[3], rot[0], rot[1], rot[2]]
            transform_dict = spartanUtils.dictFromPosQuat(trans, quat_wxyz)
            d['camera_to_world'] = transform_dict
            d['timestamp'] = depth_data['timestamps'][idx]
            d['rgb_image_filename'] = rgb_filename
            d['depth_image_filename'] = depth_filename



        spartanUtils.saveToYaml(pose_data, os.path.join(output_dir,'pose_data.yaml'))

        # extract the camera info msg

        camera_info_msg = None
        for topic, msg, t in bag.read_messages(topics=self.camera_info_topic):
            camera_info_msg = msg
            break

        
        camera_info_dict = rosUtils.camera_info_dict_from_camera_info_msg(camera_info_msg)
        
        # NOTE: currently the batch_extract_and_fuse_all_scenes.py
        # script checks for the existence of this file (camera_info.yaml) 
        # to determine if the extraction process was completed.

        spartanUtils.saveToYaml(camera_info_dict, os.path.join(output_dir,'camera_info.yaml'))
        


    @staticmethod
    def lookup_synchronized_image(query_time, timestamps):
        """
        Parameters:
            query_time: int
                the time you want to find closest match to
            

        """
        idx = np.searchsorted(timestamps, query_time)
        return min(idx, np.size(timestamps)-1)


class FusionType:
    ELASTIC_FUSION = 0
    TSDF_FUSION = 1

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


        
        # regular far out scanning poses
        pose_list = []
        pose_list.append(["Elastic Fusion", 'home'])
        pose_list.append(["Elastic Fusion", 'home_closer'])
        pose_list.append(["Elastic Fusion", 'center_right'])
        pose_list.append(["Elastic Fusion", 'right'])
        pose_list.append(["Elastic Fusion", 'right_low'])
        pose_list.append(["Elastic Fusion", 'right_low_closer'])
        pose_list.append(["Elastic Fusion", 'center_right'])
        pose_list.append(["Elastic Fusion", 'home_closer'])
        pose_list.append(["Elastic Fusion", 'center_left_closer'])
        pose_list.append(["Elastic Fusion", 'center_left_low_closer'])
        pose_list.append(["Elastic Fusion", 'left_low'])
        pose_list.append(["Elastic Fusion", 'left_mid'])
        pose_list.append(["Elastic Fusion", 'center_left_low'])
        pose_list.append(["Elastic Fusion", 'center_left_low_closer'])
        pose_list.append(["Elastic Fusion", 'center_left_closer'])
        pose_list.append(["Elastic Fusion", 'home_closer'])
        pose_list.append(["Elastic Fusion", 'top_down'])
        pose_list.append(["Elastic Fusion", 'top_down_right'])
        pose_list.append(["Elastic Fusion", 'top_down_left'])

        self.config['scan']['pose_list'] = pose_list


        # quick scan for testing purposes
        pose_list = []
        pose_list.append(["Elastic Fusion", 'home'])
        pose_list.append(["Elastic Fusion", 'home_closer'])
        pose_list.append(["Elastic Fusion", 'top_down'])
        pose_list.append(["Elastic Fusion", 'top_down_right'])
        pose_list.append(["Elastic Fusion", 'top_down_left'])
        pose_list.append(["Elastic Fusion", 'home'])
        self.config['scan']['pose_list_quick'] = pose_list



        # close up scanning
        pose_list = []
        pose_list.append(["close_up_scan", "center"])
        pose_list.append(["close_up_scan", "center_2"])
        pose_list.append(["close_up_scan", "center_3"])
        pose_list.append(["close_up_scan", "center_back_1"])
        pose_list.append(["close_up_scan", "center_back_2"])
        pose_list.append(["close_up_scan", "center_back_3"])
        pose_list.append(["close_up_scan", "center"])
        pose_list.append(["close_up_scan", "right_1"])
        pose_list.append(["close_up_scan", "right_2"])
        pose_list.append(["close_up_scan", "right_3"])
        pose_list.append(["close_up_scan", "right_4"])
        pose_list.append(["close_up_scan", "far_right_1"])
        # pose_list.append(["close_up_scan", "far_right_2"])
        # pose_list.append(["close_up_scan", "far_right_1"])
        pose_list.append(["close_up_scan", "right_1"])
        pose_list.append(["close_up_scan", "center"])
        pose_list.append(["close_up_scan", "left_1"])
        pose_list.append(["close_up_scan", "left_2"])
        pose_list.append(["close_up_scan", "left_3"])
        pose_list.append(["close_up_scan", "far_left_1"])
        pose_list.append(["close_up_scan", "far_left_2"])
        pose_list.append(["close_up_scan", "extreme_left_1"])
        pose_list.append(["close_up_scan", "extreme_left_2"])
        pose_list.append(["close_up_scan", "extreme_left_3"])
        pose_list.append(["close_up_scan", "left_1"])
        pose_list.append(["close_up_scan", "center"])

        self.config['scan']['close_up'] = pose_list


        self.config['speed'] = dict()
        self.config['speed']['scan'] = 25
        self.config['speed']['fast'] = 30
        self.config['speed']['wrist_rotation'] = 45

        self.config['spin_rate'] = 1

        
        self.config['home_pose_name'] = 'home'
        self.config['sleep_time_before_bagging'] = 3.0
        self.config['world_frame'] = 'base'
        self.config['camera_frame'] = "camera_" + self.camera_serial_number + "_rgb_optical_frame"

        self.config['sleep_time_at_each_pose'] = 0.01

        self.config["reconstruction_frame_id"] = "fusion_reconstruction"

        self.config['fusion_type'] = FusionType.TSDF_FUSION

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


    def start_bagging(self, bag_folder='logs_proto', full_path_to_bag_file=None):
        """

        :param bag_folder:
        :type bag_folder:
        :param full_path_to_bag_file: (optional) full path to where we save bag
        :type full_path_to_bag_file:
        :return:
        :rtype:
        """
        self.flushCache()

        if full_path_to_bag_file is None:
            base_path = os.path.join(spartanUtils.getSpartanSourceDir(), 'data_volume', 'pdc', bag_folder)
            log_id_name = spartanUtils.get_current_YYYY_MM_DD_hh_mm_ss()
            log_subdir = "raw"
            bagfile_directory = os.path.join(base_path, log_id_name, log_subdir)
            bagfile_name = "fusion_" + log_id_name

            full_path_to_bag_file = os.path.join(bagfile_directory, bagfile_name)
            # make bagfile directory with name
            os.system("mkdir -p " + bagfile_directory)

        # create parent folder if it doesn't exist
        parent_folder = os.path.dirname(full_path_to_bag_file)
        if not os.path.exists(parent_folder):
            os.makedirs(parent_folder)


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

        
        # sleep to allow for xtion driver compression issues to be resolved        
        rospy.sleep(self.config['sleep_time_before_bagging'])


        # get camera to world transform
        self.cache['point_cloud_to_world_stamped'] = self.getRGBOpticalFrameToWorldTransform()
        transform_stamped = self.cache['point_cloud_to_world_stamped']
        
        # build up command string
        rosbag_cmd = "rosbag record __name:=" + ROS_BAGGING_NODE_NAME
        rosbag_cmd += " -O " + full_path_to_bag_file
        for i in topics_to_bag:
            rosbag_cmd += " " + i

        # add some visibility
        print rosbag_cmd

        # start bagging
        self.bagging = True
        rosbag_proc = subprocess.Popen(rosbag_cmd, stdin=subprocess.PIPE, shell=True, cwd=parent_folder)

        rospy.loginfo("started image subscribers, sleeping for %d seconds", self.config['sleep_time_before_bagging'])
        
        return os.path.join(full_path_to_bag_file), rosbag_proc

    def _stop_bagging(self):
        """
        Stops ROS bagging
        :return:
        :rtype:
        """
        cmd = "rosnode kill /" + ROS_BAGGING_NODE_NAME
        print "cmd", cmd
        os.system(cmd)

        self.bagging = False
        self.stopImageSubscribers()

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


        self._stop_bagging()
        return StopBaggingFusionDataResponse("success")

    def handle_perform_elastic_fusion(self, req):
        ## call executable for filename
        fusion_folder = os.path.join(os.path.dirname(req.bag_filepath),"images")

        cmd = ". /opt/ros/kinetic/setup.sh && $SPARTAN_SOURCE_DIR/src/ElasticFusion/GUI/build/ElasticFusion -f -q -t 900 -d 2 -l " + fusion_folder 

        rospy.loginfo("elastic fusion cmd = %s", cmd)
        os.system(cmd)

        ply_filename = fusion_folder + ".ply"

        # of type sensor_msgs.msg.PointCloud2
        point_cloud = self.convert_ply_to_pointcloud2(PlyData.read(ply_filename))
        
        res = PerformElasticFusionResponse()
        res.fusion_output.pointcloud_filepath = ply_filename
        res.fusion_output.point_cloud = point_cloud
        res.fusion_output.point_cloud_to_world_stamped = self.cache['point_cloud_to_world_stamped']


        return res

    def convert_ply_to_pointcloud2(self, plydata):

        cloud_arr = np.zeros((len(plydata.elements[0].data), 3))
        for i in xrange(len(plydata.elements[0].data)):
            cloud_arr[i] = list(plydata.elements[0].data[i])[:3]

        return array_to_xyz_pointcloud2f(cloud_arr)

    @staticmethod
    def get_numpy_position_from_pose(pose):
        x = pose["camera_to_world"]["translation"]["x"]
        y = pose["camera_to_world"]["translation"]["y"]
        z = pose["camera_to_world"]["translation"]["z"]
        return np.asarray([x,y,z])

    @staticmethod
    def get_quaternion_from_pose(pose):
        quat = spartanUtils.getQuaternionFromDict(pose["camera_to_world"])
        x = quat["x"]
        y = quat["y"]
        z = quat["z"]
        w = quat["w"]
        return np.asarray([w,x,y,z])

    def get_joint_positions_for_pose(self, pose_data):
        """
        Looks up the joint positions for a given pose
        :param pose_data: list of strings [group_name, pose_name]. e.g.
        ["General", "q_nom"]
        :return: list[double] of joint angles
        """
        return copy.copy(self.storedPoses[pose_data[0]][pose_data[1]])

    def _move_robot_through_pose_list(self, pose_list, randomize_wrist=False, hit_original_poses=True):
        """
        Moves robot through the given list of poses
        :param pose_list: list of list of strings of form [pose_group, pose_name]
        :type pose_list:
        :param with_randomize_wrist: boolean flag on whether to randomizde wrist or not
        :type with_randomize_write:
        :return:
        :rtype:
        """

        joint_limit_safety_factor = 0.9
        wrist_joint_limit_degrees = 175.0
        safe_wrist_joint_limit_radians = (wrist_joint_limit_degrees * np.pi / 180.0) * joint_limit_safety_factor

        for pose_data in pose_list:
            print "moving to", pose_data[1]
            joint_positions = self.get_joint_positions_for_pose(pose_data)

            if randomize_wrist:
                print "before randomize wrist", joint_positions
                joint_positions_random_wrist = copy.copy(joint_positions)
                joint_positions_random_wrist[-1] = np.random.uniform(-safe_wrist_joint_limit_radians, safe_wrist_joint_limit_radians)
                print "after randomize wrist", joint_positions
                self.robotService.moveToJointPosition(joint_positions_random_wrist,
                                                      maxJointDegreesPerSecond=self.config['speed']['scan'])

                if hit_original_poses:
                    self.robotService.moveToJointPosition(joint_positions,
                                                          maxJointDegreesPerSecond=self.config['speed']['wrist_rotation'])


            else:
                self.robotService.moveToJointPosition(joint_positions,
                                                      maxJointDegreesPerSecond=self.config['speed']['scan'])

            rospy.sleep(self.config['sleep_time_at_each_pose'])


    def capture_scene(self):
        """
        This "moves around and captures all of the data needed for fusion". I.e., it:

        1. Moves the robot "home"
        2. Starts bagging all data needed for fusion
        3. Moves the robot around
        4. Stops bagging data
        5. Moves the robot back home

        This is not a service handler itself, but intended to be modularly called by service handlers.

        :return: bag_filepath, the full path to where (one) of the rosbag (fusion-*.bag) was saved
        :rtype: string

        """

        # first move home
        home_pose_joint_positions = self.storedPoses["Elastic Fusion"][self.config['home_pose_name']]
        print home_pose_joint_positions
        self.robotService.moveToJointPosition(home_pose_joint_positions, maxJointDegreesPerSecond=self.config['speed']['fast'])

        print "moved to home"


        # Start bagging for far out data collection
        # base_path = os.path.join(spartanUtils.getSpartanSourceDir(), 'data_volume', 'pdc', 'logs_proto')
        base_path = os.path.join(spartanUtils.getSpartanSourceDir(), 'data_volume', 'pdc', 'logs_shoes')
        log_id_name = spartanUtils.get_current_YYYY_MM_DD_hh_mm_ss()
        log_subdir = "raw"
        bagfile_directory = os.path.join(base_path, log_id_name, log_subdir)
        bagfile_name = "fusion_" + log_id_name + ".bag"
        full_path_to_bagfile = os.path.join(bagfile_directory, bagfile_name)

        print "moving robot through regular scan poses"
        self.start_bagging(full_path_to_bag_file=full_path_to_bagfile)
        pose_list = self.config['scan']['pose_list']
        joint_positions = self.get_joint_positions_for_pose(pose_list[0])
        self.robotService.moveToJointPosition(joint_positions,
                                              maxJointDegreesPerSecond=self.config['speed']['scan'])
        # rospy.sleep(3.0)
        self._move_robot_through_pose_list(pose_list, randomize_wrist=True, hit_original_poses=True)

        self._stop_bagging()


        # # move robot through close up scan poses
        log_subdir = "raw_close_up"
        bagfile_directory = os.path.join(base_path, log_id_name, log_subdir)
        bagfile_name = "fusion_" + log_id_name + ".bag"
        full_path_to_bagfile = os.path.join(bagfile_directory, bagfile_name)
        #
        print "moving robot through close up scan poses"
        pose_list = self.config['scan']['close_up']

        # move to first pose before we start bagging
        joint_positions = self.get_joint_positions_for_pose(pose_list[0])
        self.robotService.moveToJointPosition(joint_positions,
                                              maxJointDegreesPerSecond=self.config['speed']['scan'])

        # now start bagging and move the robot through the poses
        self.start_bagging(full_path_to_bag_file=full_path_to_bagfile)
        self._move_robot_through_pose_list(pose_list, randomize_wrist=True, hit_original_poses=True)
        # rospy.sleep(3.0)
        self._stop_bagging()
        rospy.sleep(1.0)

        # move back home
        self.robotService.moveToJointPosition(home_pose_joint_positions, maxJointDegreesPerSecond=self.config['speed']['fast'])

        return full_path_to_bagfile

    def extract_data_from_rosbag(self, bag_filepath, images_dir=None, rgb_only=False):
        """
        This wraps the ImageCapture calls to load and process the raw rosbags, to prepare for fusion.

        :param: bag_filepath, the full path to where the rosbag (fusion-*.bag) was saved
        :ptype: string

        :return: data dir, images_dir the full path to the directory where all the extracted data is saved
                            and its images subdirectory
        :rtype: two strings, separated by commas
        """

        # extract RGB and Depth images from Rosbag
        rgb_topic = self.topics_dict['rgb']
        depth_topic = self.topics_dict['depth']
        camera_info_topic = self.topics_dict['camera_info']

        if images_dir is None:
            log_dir = os.path.dirname(os.path.dirname(bag_filepath))
            processed_dir = os.path.join(log_dir, 'processed')
            images_dir = os.path.join(processed_dir, 'images')


        print "Using images_dir %s" % (images_dir)
        image_capture = ImageCapture(rgb_topic, depth_topic, camera_info_topic,
            self.config['camera_frame'], self.config['world_frame'], rgb_encoding='bgr8')
        image_capture.load_ros_bag(bag_filepath)
        image_capture.process_ros_bag(image_capture.ros_bag, images_dir, rgb_only=rgb_only)

        rospy.loginfo("Finished writing images to disk")

        return images_dir

    def handle_capture_scene(self, req):
        print "handling capture_scene"

        # Capture scene
        bag_filepath = self.capture_scene()

        response = CaptureSceneResponse()
        response.bag_filepath = bag_filepath

        rospy.loginfo("handle_capture_scene finished!")
        return response

    def handle_capture_scene_and_fuse(self, req):
        print "handling capture_scene_and_fuse"

        # Capture scene
        bag_filepath = self.capture_scene()

        # Extract images from bag
        processed_dir, images_dir = self.extract_data_from_rosbag(bag_filepath)

        if self.config['fusion_type'] == FusionType.ELASTIC_FUSION:
            # Perform fusion
            try:
                perform_elastic_fusion = rospy.ServiceProxy('perform_elastic_fusion', PerformElasticFusion)
                resp3 = perform_elastic_fusion(bag_filepath)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

            # publish the pointcloud to RVIZ
            elastic_fusion_output = resp3.fusion_output
            self.cache['fusion_output'] = elastic_fusion_output
            self.publish_pointcloud_to_rviz(elastic_fusion_output.point_cloud,
                                            self.cache['point_cloud_to_world_stamped'])

            response = CaptureSceneAndFuseResponse(elastic_fusion_output)

        elif self.config['fusion_type'] == FusionType.TSDF_FUSION:

            print "formatting data for tsdf fusion"
            tsdf_fusion.format_data_for_tsdf(images_dir)

            print "running tsdf fusion"
            tsdf_fusion.run_tsdf_fusion_cuda(images_dir)

            print "converting tsdf to ply"
            tsdf_bin_filename = os.path.join(processed_dir, 'tsdf.bin')
            tsdf_mesh_filename = os.path.join(processed_dir, 'fusion_mesh.ply')
            tsdf_fusion.convert_tsdf_to_ply(tsdf_bin_filename, tsdf_mesh_filename)

            # the response is not meaningful right now
            response = CaptureSceneAndFuseResponse()

        else:
            raise ValueError('unknown fusion type')


        # downsample data (this should be specifiable by an arg)
        print "downsampling image folder"
        linear_distance_threshold = 0.03
        angle_distance_threshold = 10 # in degrees
        FusionServer.downsample_by_pose_difference_threshold(images_dir, linear_distance_threshold, angle_distance_threshold)


        rospy.loginfo("handle_capture_scene_and_fuse finished!")
        response.fusion_output.data_folder = processed_dir
        return response

    def run_fusion_data_server(self):
        s = rospy.Service('start_bagging_fusion_data', StartBaggingFusionData, self.handle_start_bagging_fusion_data)
        s = rospy.Service('stop_bagging_fusion_data', StopBaggingFusionData, self.handle_stop_bagging_fusion_data)
        s = rospy.Service('perform_elastic_fusion', PerformElasticFusion, self.handle_perform_elastic_fusion)
        s = rospy.Service('capture_scene', CaptureScene, self.handle_capture_scene)
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


    @staticmethod
    def downsample_by_pose_difference_threshold(images_dir_full_path, linear_distance_threshold, rotation_angle_threshold):
        """
        Downsamples poses and keeps only those that are sufficiently apart
        :param images_dir_full_path:
        :type images_dir_full_path:
        :param linear_distance_threshold: threshold on the translation, in meters
        :type linear_distance_threshold:
        :param rotation_angle_threshold: threshold on the angle between the rotations, in degrees
        :type rotation_angle_threshold:
        :return:
        :rtype:
        """
        pose_yaml = os.path.join(images_dir_full_path, "pose_data.yaml")
        pose_dict = spartanUtils.getDictFromYamlFilename(pose_yaml)

        images_dir_temp_path = os.path.join(os.path.dirname(images_dir_full_path), 'images_temp')
        if not os.path.isdir(images_dir_temp_path):
            os.makedirs(images_dir_temp_path)
        
        
        previous_pose_pos = FusionServer.get_numpy_position_from_pose(pose_dict[0])
        previous_pose_quat = FusionServer.get_quaternion_from_pose(pose_dict[0])

        print "Using downsampling by pose difference threshold... "
        

        num_kept_images    = 0
        num_deleted_images = 0

        pose_dict_downsampled = dict()

        img_indices = pose_dict.keys()
        img_indices.sort()
        num_original_images = len(img_indices)

        for i in img_indices:
            single_frame_data = pose_dict[i]
            this_pose_pos = FusionServer.get_numpy_position_from_pose(single_frame_data)
            this_pose_quat = FusionServer.get_quaternion_from_pose(single_frame_data)

            linear_distance = np.linalg.norm(this_pose_pos - previous_pose_pos)

            rotation_distance = spartanUtils.compute_angle_between_quaternions(this_pose_quat,
                previous_pose_quat)


            if i == 0:
                keep_image = True
                num_kept_images += 1
            elif (linear_distance > linear_distance_threshold) or (np.rad2deg(rotation_distance) > rotation_angle_threshold):
                previous_pose_pos = this_pose_pos
                previous_pose_quat = this_pose_quat
                num_kept_images += 1
            else:
                # the pose wasn't sufficiently different
                continue

            # if we have gotten here, then move the images over to the new directory
            pose_dict_downsampled[i] = single_frame_data
            rgb_filename = os.path.join(images_dir_full_path, single_frame_data['rgb_image_filename'])
            rgb_filename_temp = os.path.join(images_dir_temp_path, single_frame_data['rgb_image_filename'])

            shutil.move(rgb_filename, rgb_filename_temp)

            depth_filename = os.path.join(images_dir_full_path, single_frame_data['depth_image_filename'])
            depth_filename_temp = os.path.join(images_dir_temp_path, single_frame_data['depth_image_filename'])
            shutil.move(depth_filename, depth_filename_temp)
                # # delete pose from posegraph
                # del posegraph_list[i-num_deleted_images]
                # num_deleted_images += 1

        
        # write downsamples pose_data.yaml (forward kinematics)
        spartanUtils.saveToYaml(pose_dict_downsampled, os.path.join(images_dir_temp_path,'pose_data.yaml'))

        # remove old images
        shutil.move(os.path.join(images_dir_full_path, 'camera_info.yaml'), os.path.join(images_dir_temp_path, 'camera_info.yaml'))
        shutil.rmtree(images_dir_full_path)

        print "renaming %s to %s " %(images_dir_temp_path, images_dir_full_path)

        # rename temp images to images
        os.rename(images_dir_temp_path, images_dir_full_path)

        print "Previously: %d images" %(num_original_images)
        print "After: %d images" %(num_kept_images)
