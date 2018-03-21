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

    def process_ros_bag(self, bag, output_dir):

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
        spartanUtils.saveToYaml(camera_info_dict, os.path.join(output_dir,'camera_info.yaml'))
        


    @staticmethod
    def lookup_synchronized_image(query_time, timestamps):
        """
        Parameters:
            query_time: int
                the time you want to find closest match to
            

        """
        idx = np.searchsorted(timestamps, query_time)
        return min(idx, np.size(timestamps))

         

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
        self.config['camera_frame'] = "camera_" + self.camera_serial_number + "_rgb_optical_frame"

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

        bagfile_name = "fusion_" + str(time.time())
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

        
        # sleep to allow for xtion driver compression issues to be resolved        
        rospy.sleep(self.config['sleep_time_before_bagging'])


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

        rospy.loginfo("started image subscribers, sleeping for %d seconds", self.config['sleep_time_before_bagging'])
        
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
        fusion_folder = os.path.join(os.path.dirname(req.bag_filepath),"images")

        cmd = ". /opt/ros/kinetic/setup.sh && $SPARTAN_SOURCE_DIR/src/ElasticFusion/GUI/build/ElasticFusion -f -q -t 900 -d 2 -l " + fusion_folder 

        rospy.loginfo("elastic fusion cmd = %s", cmd)
        os.system(cmd)

        ply_filename = fusion_folder + ".ply"

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

    def get_numpy_position_from_pose(self, pose):
        x = pose["camera_to_world"]["translation"]["x"]
        y = pose["camera_to_world"]["translation"]["y"]
        z = pose["camera_to_world"]["translation"]["z"]
        return np.asarray([x,y,z])

    def downsample_by_pose_difference_threshold(self, images_dir_full_path, threshold):
        pose_yaml = os.path.join(images_dir_full_path, "pose_data.yaml")
        pose_dict = spartanUtils.getDictFromYamlFilename(pose_yaml)

        images_dir_temp_path = os.path.join(os.path.dirname(images_dir_full_path), 'images_temp')
        if not os.path.isdir(images_dir_temp_path):
            os.makedirs(images_dir_temp_path)


        posegraph_filename = images_dir_full_path+".posegraph"
        with open(posegraph_filename) as f:
            posegraph_list = f.readlines()
        
        previous_pose = self.get_numpy_position_from_pose(pose_dict[0])

        print "Using downsampling by pose difference threshold... "
        print "Previously: ", len(pose_dict), " images"

        num_kept_images    = 0
        num_deleted_images = 0

        for i in range(0,len(pose_dict)):
            single_frame_data = pose_dict[i]
            this_pose = self.get_numpy_position_from_pose(pose_dict[i])

            if i == 0:
                keep_image = True
                num_kept_images += 1
            elif np.linalg.norm(previous_pose - this_pose) > threshold:
                previous_pose = this_pose
                num_kept_images += 1
            else:
                # delete pose from forward kinematics
                del pose_dict[i]
                continue

            # if we have gotten here, then move the images over to the new directory

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
        spartanUtils.saveToYaml(pose_dict, os.path.join(images_dir_temp_path,'pose_data.yaml'))

        # remove old images
        shutil.move(os.path.join(images_dir_full_path, 'camera_info.yaml'), os.path.join(images_dir_temp_path, 'camera_info.yaml'))
        shutil.rmtree(images_dir_full_path)

        print "renaming %s to %s " %(images_dir_temp_path, images_dir_full_path)

        # rename temp images to images
        os.rename(images_dir_temp_path, images_dir_full_path)

        print "After: ", num_kept_images, " images"


    def handle_capture_scene_and_fuse(self, req):
        # Start bagging with own srv call
        print "handling capture_scene_and_fuse"

        # first move home
        home_pose_joint_positions = self.storedPoses[self.config['scan']['pose_group']][self.config['home_pose_name']]
        print home_pose_joint_positions
        self.robotService.moveToJointPosition(home_pose_joint_positions, maxJointDegreesPerSecond=self.config['speed']['fast'])

        print "moved to home"

        try:
            start_bagging_fusion_data = rospy.ServiceProxy('start_bagging_fusion_data', StartBaggingFusionData)
            resp1 = start_bagging_fusion_data()
            bag_filepath = resp1.bag_filepath
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Move robot around
        for poseName in self.config['scan']['pose_list']:
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


        # extract RGB and Depth images from Rosbag
        rgb_topic = self.topics_dict['rgb']
        depth_topic = self.topics_dict['depth']
        camera_info_topic = self.topics_dict['camera_info']
        
        output_dir = os.path.join(os.path.dirname(bag_filepath), 'images')
        image_capture = ImageCapture(rgb_topic, depth_topic, camera_info_topic,
        self.config['camera_frame'], self.config['world_frame'], rgb_encoding='bgr8')
        image_capture.load_ros_bag(bag_filepath)
        image_capture.process_ros_bag(image_capture.ros_bag, output_dir)

        rospy.loginfo("Finished writing images to disk")

        # Perform fusion
        try:
            perform_elastic_fusion = rospy.ServiceProxy('perform_elastic_fusion', PerformElasticFusion)
            resp3 = perform_elastic_fusion(resp1.bag_filepath)           
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # downsample data (this should be specifiable by an arg)
        print "output_dir is", output_dir  
        self.downsample_by_pose_difference_threshold(output_dir, threshold=0.03)

        # publish the pointcloud to RVIZ
        elastic_fusion_output = resp3.elastic_fusion_output
        self.cache['fusion_output'] = elastic_fusion_output
        self.publish_pointcloud_to_rviz(elastic_fusion_output.point_cloud, self.cache['point_cloud_to_world_stamped'])

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