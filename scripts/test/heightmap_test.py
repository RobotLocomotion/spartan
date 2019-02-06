import numpy as np
import spartan.perception.utils as perception_utils
import spartan.utils.utils as utils
from spartan.perception.heightmap import HeightMap
import ros
import rospy
import ros_numpy
import rosbag
import sensor_msgs


def test():

    depth_img_filename = "/home/manuelli/spartan/data_volume/pdc/logs_proto/2018-11-16-22-22-45/processed/images/000000_depth.png"
    pose_data_filename = "/home/manuelli/spartan/data_volume/pdc/logs_proto/2018-11-16-22-22-45/processed/images/pose_data.yaml"
    pose_data = utils.getDictFromYamlFilename(pose_data_filename)
    camera_pose = pose_data[0]


    ros_bag_filename = "/home/manuelli/spartan/data_volume/spartan/rosbag/shoe_globalwin_brown_down_right_2018-12-04-00-31-51.bag"
    ros_bag = rosbag.Bag(ros_bag_filename, "r")

    pointcloud_topic = "/camera_carmine_1/depth_registered/points"
    pointcloud_msg = None


    def copy_msg(msg):
        new_msg = sensor_msgs.msg.PointCloud2()
        new_msg.header = msg.header
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = msg.point_step
        new_msg.row_step = msg.row_step
        new_msg.data = msg.data
        new_msg.is_dense = msg.is_dense
        return new_msg


    topics = [pointcloud_topic]
    counter = 0
    for topic, msg, t in ros_bag.read_messages(topics=topics):
        #     print topic
        counter += 1
        if counter < 50:
            continue
        if topic == pointcloud_topic:
            pointcloud_msg = copy_msg(msg)
            break

    ros_bag.close()
    print pointcloud_msg.header
    pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud_msg, remove_nans=True)

    print pc.shape

    # use rosrun tf tf_echo /base /camera_carmine_1_rgb_optical_frame

    translation = [0.593, 0.027, 0.588]
    quat_xyzw = [0.711, -0.702, 0.005, -0.041]

    d = dict()
    d['translation'] = dict()
    d['translation']['x'] = translation[0]
    d['translation']['y'] = translation[1]
    d['translation']['z'] = translation[2]

    d['quaternion'] = dict()
    d['quaternion']['w'] = quat_xyzw[3]
    d['quaternion']['x'] = quat_xyzw[0]
    d['quaternion']['y'] = quat_xyzw[1]
    d['quaternion']['z'] = quat_xyzw[2]

    T_H_pc = utils.homogenous_transform_from_dict(d)
    print T_H_pc

    pc_world_frame = perception_utils.transform_pointcloud(pc, T_H_pc)

    hm = HeightMap.make_default()
    hm.insert_pointcloud_into_heightmap(pc, T_H_pc)

    pc_hm = hm.heightmap_to_pointcloud()
    pc2 = ros_numpy.point_cloud2.array_to_pointcloud2(pc_hm)

    pc2.header.stamp = rospy.Time.now()
    pc2.header.frame_id = "base"

    # now publish this in loop
    pub = rospy.Publisher('heightmap_pointcloud', sensor_msgs.msg.PointCloud2,
                          queue_size=1)

    pub2 = rospy.Publisher('heightmap_pointcloud_original', sensor_msgs.msg.PointCloud2,
                          queue_size=1)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        print "publishing pointcloud"
        pub.publish(pc2)
        pub2.publish(pointcloud_msg)
        r.sleep()


if __name__ == "__main__":
    rospy.init_node("heightmap_test")
    test()


