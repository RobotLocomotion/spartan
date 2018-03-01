import os
import sys
import time
import numpy as np


import spartan.utils.cameraposes as cameraposes

spartan_source_dir = os.getenv("SPARTAN_SOURCE_DIR")
Open3D_python_lib_dir = os.path.join(spartan_source_dir, "src/Open3D/build/lib")

# import spartan.utils.cameraposes as cameraposes

sys.path.append(Open3D_python_lib_dir)

import py3d
from py3d import *
import matplotlib.pyplot as plt


log_dir = os.path.join(spartan_source_dir, "sandbox/open3d/kuka_scene")

def draw_rgbd_images():
    color_image_filename = os.path.join(log_dir, "images", "000000_rgb.png")
    depth_image_filename = os.path.join(log_dir, "images", "000000_depth.png")

    print "color_image_filename: ", color_image_filename
    color_raw = py3d.read_image(color_image_filename)
    print "loaded rgb image"
    
    print "depth_image_filename: ", depth_image_filename
    depth_raw = py3d.read_image(depth_image_filename)
    print "loaded depth image"
    rgbd_image = py3d.create_rgbd_image_from_color_and_depth(
        color_raw, depth_raw)

    plt.subplot(1, 2, 1)
    plt.title('grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()

    pcd = py3d.create_point_cloud_from_rgbd_image(rgbd_image,
            PinholeCameraIntrinsic.prime_sense_default)
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    py3d.draw_geometries([pcd])
    

# def load_camera_poses():
#     pass

# def rgbd_integration():
#     volume = ScalableTSDFVolume(voxel_length = 4.0 / 512.0,
#             sdf_trunc = 0.04, with_color = True)

#     for i in range(len(camera_poses)):
#         print("Integrate {:d}-th image into the volume.".format(i))
#         color = read_image("../../TestData/RGBD/color/{:05d}.jpg".format(i))
#         depth = read_image("../../TestData/RGBD/depth/{:05d}.png".format(i))
#         rgbd = create_rgbd_image_from_color_and_depth(color, depth,
#                 depth_trunc = 4.0, convert_rgb_to_intensity = False)
#         volume.integrate(rgbd, PinholeCameraIntrinsic.prime_sense_default,
#                 np.linalg.inv(camera_poses[i].pose))

#     print("Extract a triangle mesh from the volume and visualize it.")
#     mesh = volume.extract_triangle_mesh()
#     mesh.compute_vertex_normals()
#     draw_geometries([mesh])

def convert_image_idx_to_padded_string(n, numCharacters=6):
    """
    Converts the integer n to a padded string with leading zeros
    """
    t = str(n)
    return t.rjust(numCharacters, '0')

def rgbd_integration(log_dir):
    posegraph_file = os.path.join(log_dir, "posegraph.posegraph")
    camera_poses = cameraposes.CameraPoses(posegraph_file)
    num_images = len(camera_poses.poses)

    width = 640
    height = 480
    fx = 539.075603
    cx = 316.229489
    fy = 539.782595
    cy = 236.154597
    camera_intrinsics = PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

    
    start_time = time.time()
    volume = ScalableTSDFVolume(voxel_length = 4.0 / 512.0,
            sdf_trunc = 0.04, with_color = True)

    # num_images = 500 # stay with a small number of images for now
    idx_list = [0,500]
    idx_list = range(0,num_images)
    for idx in idx_list:
        print("Integrate {:d}-th image into the volume.".format(idx))
        prefix = convert_image_idx_to_padded_string(idx)
        color_image_filename = os.path.join(log_dir, 'images', prefix + "_rgb.png")
        depth_image_filename = os.path.join(log_dir, 'images', prefix + "_depth.png")

        
        color = py3d.read_image(color_image_filename)
        depth = py3d.read_image(depth_image_filename)

        rgbd = create_rgbd_image_from_color_and_depth(color, depth, convert_rgb_to_intensity=False, depth_trunc=1.0)

        camera_to_reconstruction = camera_poses.getCameraPose(idx)
        reconstruction_to_camera = np.linalg.inv(camera_to_reconstruction)
        

        volume.integrate(rgbd, camera_intrinsics, reconstruction_to_camera)

    print "integrating images into tsdf took %d seconds" %(time.time() - start_time)

    mesh_start_time = time.time()
    print("Extract a triangle mesh from the volume and visualize it.")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    print("Done extracting triangle mesh")
    
    print "extracting triangle mesh took %d seconds" %(time.time() - mesh_start_time)

    print "reconstruction took %d seconds " %(time.time() - start_time)
    draw_geometries([mesh])


if __name__ == "__main__":
    # draw_rgbd_images()
    rgbd_integration(log_dir)
