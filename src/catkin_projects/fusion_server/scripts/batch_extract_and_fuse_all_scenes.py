#!/usr/bin/python

import os
import rospy
import time
import gc
import multiprocessing as mp
import resource
import shutil
import argparse


from fusion_server.fusion import FusionServer
import fusion_server.tsdf_fusion as tsdf_fusion

import spartan.utils.utils as spartanUtils

# this is necessary to avoid getting an error about /unnamed/tf2_server in the construction
# of the tf2buffer in ROS
fs = FusionServer()

LINEAR_DISTANCE_THRESHOLD = 0.03 # 3cm
ANGLE_DISTANCE_THRESHOLD = 10 # 10 degrees

def mem():
    print('Memory usage         : % 2.2f MB' % round(
        resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.0,1)
    )

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

def extract_data_from_rosbag(bag_filepath, images_dir):
    images_dir = fs.extract_data_from_rosbag(bag_filepath, images_dir=images_dir)



def extract_and_fuse_single_scene(log_full_path, downsample=True,
                        linear_distance_threshold=LINEAR_DISTANCE_THRESHOLD,
                        angle_distance_threshold=ANGLE_DISTANCE_THRESHOLD):
    """
    Extracts all the images from the rosbag.
    :param log_full_path:
    :param downsample: whether or not to downsample
    :param linear_distance_threshold: threshold used in downsampling
    :param angle_distance_threshold: threshold used in downsampling
    :return:
    """

    print "extracting and fusing scene:", log_full_path 
    log = os.path.split(log_full_path)[-1]

    mem()

    processed_dir = os.path.join(log_full_path, 'processed')
    images_dir = os.path.join(processed_dir, 'images')
    raw_dir = os.path.join(log_full_path, 'raw')
    raw_close_up_dir = os.path.join(log_full_path, 'raw_close_up')
    bag_filepath = os.path.join(log_full_path, 'raw', 'fusion_' + log + '.bag')
    bag_close_up_filepath = os.path.join(raw_close_up_dir, 'fusion_' + log + '.bag')
    pose_data_filename = os.path.join(images_dir, 'pose_data.yaml')
    metadata_filename = os.path.join(images_dir, 'metadata.yaml')


    if not already_extracted_rosbag(log_full_path):
        print "extracting", bag_filepath

        # run this memory intensive step in a process, to allow
        # it to release memory back to the operating system
        # when finished
        proc = mp.Process(target=extract_data_from_rosbag, args=(bag_filepath,images_dir))
        proc.start()
        proc.join()

        print "finished extracting", bag_filepath
        # counter_new_extracted += 1
    else:
        print "already extracted", bag_filepath
    
    # we need to free some memory
    gc.collect()
    mem()


    if not already_ran_tsdf_fusion(log_full_path):
        print "preparing for tsdf_fusion for", log_full_path
        tsdf_fusion.format_data_for_tsdf(images_dir)
        gc.collect()

        print "running tsdf fusion"
        tsdf_fusion.run_tsdf_fusion_cuda(images_dir)
        gc.collect()

        print "converting tsdf to ply"
        tsdf_bin_filename = os.path.join(processed_dir, 'tsdf.bin')
        tsdf_mesh_filename = os.path.join(processed_dir, 'fusion_mesh.ply')
        tsdf_fusion.convert_tsdf_to_ply(tsdf_bin_filename, tsdf_mesh_filename)
        # counter_new_fused += 1
    else:
        print "already ran tsdf_fusion for", log_full_path

    if not already_downsampled(log_full_path) and downsample:
        print "downsampling image folder"
        
        # fs = FusionServer()
        fs.downsample_by_pose_difference_threshold(images_dir, linear_distance_threshold, angle_distance_threshold)
        # counter_new_downsampled += 1

        # create metadata.yaml file if it doesn't yet exist
        if not os.path.exists(metadata_filename):
            pose_data = spartanUtils.getDictFromYamlFilename(pose_data_filename)
            original_image_indices = pose_data.keys()

            metadata = dict()
            metadata['original_image_indices'] = original_image_indices
            metadata['close_up_image_indices'] = []
            spartanUtils.saveToYaml(metadata, metadata_filename)


    else:
        print "already downsampled for", log_full_path


    def already_extracted_close_up_log():

        if not os.path.exists(metadata_filename):
            return False

        metadata_temp = spartanUtils.getDictFromYamlFilename(metadata_filename)
        return (len(metadata_temp['close_up_image_indices']) > 0)

    # extract the up close up log if it exists
    if os.path.exists(bag_close_up_filepath) and not already_extracted_close_up_log():
        print "\n\n-----------extracting close up images----------\n\n"
        print "extracting", bag_close_up_filepath
        close_up_temp_dir = os.path.join(processed_dir, 'images_close_up')


        # run this memory intensive step in a process, to allow
        # it to release memory back to the operating system
        # when finished
        proc = mp.Process(target=extract_data_from_rosbag, args=(bag_close_up_filepath, close_up_temp_dir))
        proc.start()
        proc.join()

        print "downsampling image folder %s" %(close_up_temp_dir)
        if downsample:
            fs.downsample_by_pose_difference_threshold(close_up_temp_dir, linear_distance_threshold, angle_distance_threshold)

        pose_data_close_up_filename = os.path.join(close_up_temp_dir, 'pose_data.yaml')
        pose_data_close_up = spartanUtils.getDictFromYamlFilename(pose_data_close_up_filename)

        pose_data = spartanUtils.getDictFromYamlFilename(pose_data_filename)
        original_image_indices = pose_data.keys()
        largest_original_image_idx = max(original_image_indices)

        close_up_image_indices = []

        print "largest_original_image_idx", largest_original_image_idx


        for img_idx, data in pose_data_close_up.iteritems():
            # just to give some buffer and avoid edge cases in some checks
            img_idx_new = img_idx + largest_original_image_idx + 4
            rgb_filename = "%06i_%s.png" % (img_idx_new, "rgb")
            depth_filename = "%06i_%s.png" % (img_idx_new, "depth")

            rgb_filename_prev = data['rgb_image_filename']
            depth_filename_prev = data['depth_image_filename']

            data['rgb_image_filename'] = rgb_filename
            data['depth_image_filename'] = depth_filename

            pose_data[img_idx_new] = data
            close_up_image_indices.append(img_idx_new)

            # move images around
            shutil.move(os.path.join(close_up_temp_dir, rgb_filename_prev),
                        os.path.join(images_dir, rgb_filename))

            shutil.move(os.path.join(close_up_temp_dir, depth_filename_prev),
                        os.path.join(images_dir, depth_filename))


        shutil.rmtree(close_up_temp_dir)

        original_image_indices.sort()
        close_up_image_indices.sort()
        spartanUtils.saveToYaml(pose_data, pose_data_filename)

        metadata = dict()
        metadata['normal_image_indices'] = original_image_indices
        metadata['close_up_image_indices'] = close_up_image_indices
        spartanUtils.saveToYaml(metadata, metadata_filename)

    else:
        if os.path.exists(bag_close_up_filepath):
            print "already extracted close up log, skipping"
        else:
            print "raw_close_up doesn't exist, skipping"



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--logs_dir", type=str, required=False)
    args = parser.parse_args()

    start = time.time()
    

    if args.logs_dir:
        logs_proto_path = args.logs_dir
    else:
        # logs_proto_path = os.path.join(spartanUtils.getSpartanSourceDir(), 'data_volume', 'pdc', 'logs_special', 'static_scenes')
        logs_proto_path = os.path.join(spartanUtils.getSpartanSourceDir(), 'data_volume', 'pdc', 'logs_shoes')

    logs_proto_list = sorted(os.listdir(logs_proto_path))

    counter_new_extracted   = 0
    counter_new_fused       = 0
    counter_new_downsampled = 0

    for log in logs_proto_list:
        log_full_path = os.path.join(logs_proto_path, log)
        extract_and_fuse_single_scene(log_full_path, downsample=True)

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





