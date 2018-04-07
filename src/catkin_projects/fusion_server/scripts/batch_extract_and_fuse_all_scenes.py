#!/usr/bin/python

import os
import rospy
import time

from fusion_server.fusion import FusionServer
import fusion_server.tsdf_fusion as tsdf_fusion

import spartan.utils.utils as spartanUtils


def already_extracted_rosbag(log_full_path):
    images_dir = os.path.join(log_full_path, 'processed', 'images')
    file_to_check = os.path.join(images_dir, 'camera_info.yaml')
    return os.path.exists(file_to_check)
        
def already_ran_tsdf_fusion(log_full_path):
    processed_dir = os.path.join(log_full_path, 'processed')
    file_to_check = os.path.join(processed_dir, 'fusion_mesh.ply')
    return os.path.exists(file_to_check)

def already_downsampled(log_full_path):
    images_dir = os.path.join(log_full_path, 'processed', 'images')
    file_to_check_1 = os.path.join(images_dir, "000000_rgb.png")
    file_to_check_2 = os.path.join(images_dir, "000001_rgb.png")
    return not (os.path.exists(file_to_check_1) and os.path.exists(file_to_check_2))
 
if __name__ == "__main__":

    start = time.time()

    fs = FusionServer()
    
    logs_proto_path = os.path.join(spartanUtils.getSpartanSourceDir(), 'data_volume', 'pdc', 'logs_proto')

    logs_proto_list = sorted(os.listdir(logs_proto_path))

    counter_new_extracted   = 0
    counter_new_fused       = 0
    counter_new_downsampled = 0

    for log in logs_proto_list:

        log_full_path = os.path.join(logs_proto_path, log)

        print log_full_path

        if not already_extracted_rosbag(log_full_path):
            print "extracting", log_full_path
            bag_filepath = os.path.join(log_full_path, 'raw', 'fusion_'+log+'.bag')
            processed_dir, images_dir = fs.extract_data_from_rosbag(bag_filepath)
            counter_new_extracted += 1
        else:
            print "already extracted", log_full_path
            processed_dir = os.path.join(log_full_path, 'processed')
            images_dir    = os.path.join(processed_dir, 'images')

        if not already_ran_tsdf_fusion(log_full_path):
            print "preparing for tsdf_fusion for", log_full_path
            tsdf_fusion.format_data_for_tsdf(images_dir)

            print "running tsdf fusion"
            tsdf_fusion.run_tsdf_fusion_cuda(images_dir)

            print "converting tsdf to ply"
            tsdf_bin_filename = os.path.join(processed_dir, 'tsdf.bin')
            tsdf_mesh_filename = os.path.join(processed_dir, 'fusion_mesh.ply')
            tsdf_fusion.convert_tsdf_to_ply(tsdf_bin_filename, tsdf_mesh_filename)
            counter_new_fused += 1
        else:
            print "already ran tsdf_fusion for", log_full_path

        if not already_downsampled(log_full_path):
            print "downsampling image folder"
            fs.downsample_by_pose_difference_threshold(images_dir, threshold=0.03)
            counter_new_downsampled += 1
        else:
            print "already downsampled for", log_full_path


    print "finished extracting and fusing all logs in logs_proto"

    print "SUMMARY:"
    print "number new logs extracted:  ", counter_new_extracted
    print "number new logs fused:      ", counter_new_fused
    print "number new logs downsampled:", counter_new_downsampled

    end = time.time()
    hours, rem = divmod(end-start, 3600)
    minutes, seconds = divmod(rem, 60)
    time_string = "{:0>2}:{:0>2}:{:05.2f}".format(int(hours),int(minutes),seconds)
    print "total time:               ",  time_string
    print "(hours, minutes, seconds with two decimals)"





