#!/usr/bin/python
import numpy as np
import os

from fusion_server.fusion import ImageCapture
import fusion_server.tsdf_fusion as tsdf_fusion

# rgb_topic = "/camera_carmine_1/rgb/image_rect_color"
# depth_topic = "/camera_carmine_1/depth_registered/sw_registered/image_rect"
# camera_info_topic = "/camera_carmine_1/rgb/camera_info"
# camera_frame = "camera_carmine_1_rgb_optical_frame"
# world_frame = "base"

data_folder = "/home/manuelli/spartan/data_volume/pdc/logs_special/static_scenes/2018-05-07-22-04-57"

processed_dir = os.path.join(data_folder, 'processed')
images_dir = os.path.join(processed_dir, 'images')
# bag_filepath = os.path.join(data_folder, "fusion.bag")

def extract_images_from_rosbag():
    image_capture = ImageCapture(rgb_topic, depth_topic, camera_info_topic,
    camera_frame, world_frame, rgb_encoding='bgr8')
    image_capture.load_ros_bag(bag_filepath)

    output_dir = image_folder
    image_capture.process_ros_bag(image_capture.ros_bag, output_dir)

def main():
    print "preparing for tsdf_fusion for"
    tsdf_fusion.format_data_for_tsdf(images_dir)

    print "running tsdf fusion"
    tsdf_fusion.run_tsdf_fusion_cuda(images_dir)

    print "converting tsdf to ply"
    tsdf_bin_filename = os.path.join(processed_dir, 'tsdf.bin')
    tsdf_mesh_filename = os.path.join(processed_dir, 'fusion_mesh.ply')
    tsdf_fusion.convert_tsdf_to_ply(tsdf_bin_filename, tsdf_mesh_filename)


if __name__ == "__main__":
    main()


    # process the rosbag and 
