#!/usr/bin/python
import os
import shutil
import numpy as np
import math
import yaml
import time
from skimage import measure
from plyfile import PlyData, PlyElement
import array

import spartan.utils.utils as spartan_utils


def format_data_for_tsdf(image_folder):
    """
    Processes the data into the format needed for tsdf-fusion algorithm
    """

    # image_folder = os.path.join(data_folder, 'images')
    camera_info_yaml = os.path.join(image_folder, "camera_info.yaml")


    camera_info = spartan_utils.getDictFromYamlFilename(camera_info_yaml)

    K_matrix = camera_info['camera_matrix']['data']
    # print K_matrix
    n = K_matrix[0]

    def sci(n):
      return "{:.8e}".format(n)

    camera_intrinsics_out = os.path.join(image_folder,"camera-intrinsics.txt")
    with open(camera_intrinsics_out, 'w') as the_file:
        the_file.write(" "+sci(K_matrix[0])+"    "+sci(K_matrix[1])+"    "+sci(K_matrix[2])+"   \n")
        the_file.write(" "+sci(K_matrix[3])+"    "+sci(K_matrix[4])+"    "+sci(K_matrix[5])+"   \n")
        the_file.write(" "+sci(K_matrix[6])+"    "+sci(K_matrix[7])+"    "+sci(K_matrix[8])+"   \n") 


    ### HANDLE POSES

    pose_data_yaml = os.path.join(image_folder, "pose_data.yaml")
    with open(pose_data_yaml, 'r') as stream:
        try:
            pose_data_dict = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    print pose_data_dict[0]

    for i in pose_data_dict:
        # print i
        # print pose_data_dict[i]
        pose4 = spartan_utils.homogenous_transform_from_dict(pose_data_dict[i]['camera_to_world'])
        depth_image_filename = pose_data_dict[i]['depth_image_filename']
        prefix = depth_image_filename.split("depth")[0]
        print prefix
        pose_file_name = prefix+"pose.txt"
        pose_file_full_path = os.path.join(image_folder, pose_file_name)
        with open(pose_file_full_path, 'w') as the_file:
            the_file.write(" "+sci(pose4[0,0])+"     "+sci(pose4[0,1])+"     "+sci(pose4[0,2])+"     "+sci(pose4[0,3])+"    \n")
            the_file.write(" "+sci(pose4[1,0])+"     "+sci(pose4[1,1])+"     "+sci(pose4[1,2])+"     "+sci(pose4[1,3])+"    \n")
            the_file.write(" "+sci(pose4[2,0])+"     "+sci(pose4[2,1])+"     "+sci(pose4[2,2])+"     "+sci(pose4[2,3])+"    \n")
            the_file.write(" "+sci(pose4[3,0])+"     "+sci(pose4[3,1])+"     "+sci(pose4[3,2])+"     "+sci(pose4[3,3])+"    \n")



def run_tsdf_fusion_cuda(image_folder, output_dir=None, voxel_grid_origin_x=0.4,
    voxel_grid_origin_y=-0.3, voxel_grid_origin_z=-0.2, voxel_size=0.0025,
    voxel_grid_dim_x=240, voxel_grid_dim_y=320, voxel_grid_dim_z=280, fast_tsdf_settings=False):
    """
    Simple wrapper to call the tsdf-fusion executable with the desired args
    """
    if output_dir is None:
        output_dir = os.path.dirname(image_folder)
        print "output_dir: ", output_dir

    camera_intrinsics_file = os.path.join(image_folder, 'camera-intrinsics.txt')

    spartan_source_dir = spartan_utils.getSpartanSourceDir()
    tsdf_fusion_dir = os.path.join(spartan_source_dir, "src", "tsdf-fusion")
    tsdf_executable = os.path.join(tsdf_fusion_dir, 'demo')
    cmd = "cd %s && %s %s %s" %(tsdf_fusion_dir, tsdf_executable, image_folder, camera_intrinsics_file)


    if fast_tsdf_settings:
        voxel_size = 0.005
        voxel_grid_dim_x = 200
        voxel_grid_dim_y = 200
        voxel_grid_dim_z = 150
    
    cmd += " " + str(voxel_size)
    cmd += " " + str(voxel_grid_dim_x)
    cmd += " " + str(voxel_grid_dim_y)
    cmd += " " + str(voxel_grid_dim_z)

    cmd += " " + str(voxel_grid_origin_x)
    cmd += " " + str(voxel_grid_origin_y)
    cmd += " " + str(voxel_grid_origin_z)

    print "cmd:\n", cmd

    start_time = time.time()
    os.system(cmd)
    elapsed = time.time() - start_time

    tsdf_bin = os.path.join(image_folder, 'tsdf.bin')
    tsdf_ply = os.path.join(image_folder, 'tsdf.ply')

    tsdf_bin_new = os.path.join(output_dir, 'tsdf.bin')
    tsdf_ply_new = os.path.join(output_dir, 'fusion_pointcloud.ply')
    
    shutil.move(tsdf_bin, tsdf_bin_new)
    shutil.move(tsdf_ply, tsdf_ply_new)

    print "tsdf-fusion took %d seconds" %(elapsed)



def convert_tsdf_to_ply(tsdf_bin_filename, tsdf_mesh_filename):
    """
    Converts the tsdf binary file to a mesh file in ply format

    The indexing in the tsdf is
    (x,y,z) <--> (x + y * dim_x + z * dim_x * dim_y)
    """
    start_time = time.time()
    fin = open(tsdf_bin_filename, "rb")

    tsdfHeader = array.array("f")  # f is the typecode for float32
    tsdfHeader.fromfile(fin, 8)
    # print tsdfHeader
    # print type(tsdfHeader)

    voxelGridDim = tsdfHeader[0:3]
    voxelGridDim = np.asarray(voxelGridDim, dtype=np.int)
    voxelGridOrigin = tsdfHeader[3:6]
    voxelSize = tsdfHeader[6]
    truncMargin = tsdfHeader[7]

    dim_x = voxelGridDim[0]
    dim_y = voxelGridDim[1]
    dim_z = voxelGridDim[2]


    headerSize = 8
    tsdf_vec = np.fromfile(tsdf_bin_filename, np.float32)
    tsdf_vec = tsdf_vec[headerSize:]
    tsdf = np.reshape(tsdf_vec, voxelGridDim, order='F') # reshape using Fortran order

    # for loop version of the above reshape operation
    # for x in xrange(0, dim_x):
    #     for y in xrange(0, dim_y):
    #         for z in xrange(0, dim_z):
    #             idx = x + y * dim_x + z * dim_x * dim_y
    #             tsdf[x,y,z] = tsdf_vec[idx]


    print "tsdf.shape:", tsdf.shape
    print "voxelGridDim: ", voxelGridDim
    print "voxeGridOrigin: ", voxelGridOrigin
    print "tsdf.shape:", tsdf.shape

    verts, faces, normals, values = measure.marching_cubes_lewiner(tsdf, spacing=[voxelSize]*3)


    print "type(verts): ", type(verts)
    print "verts.shape: ", verts.shape
    print "faces.shape:", faces.shape

    print "np.max(verts[:,0]): ", np.max(verts[:,0])
    print "np.min(verts[:,0]): ", np.min(verts[:, 0])


    print "verts[0,:] = ", verts[0,:]
    print "faces[0,:] = ", faces[0,:]

    # transform from voxel coordinates to camera coordinates
    # note x and y are flipped in the output of marching_cubes
    mesh_points = np.zeros_like(verts)
    # mesh_points = verts
    mesh_points[:,0] = voxelGridOrigin[0] + verts[:,0]
    mesh_points[:,1] = voxelGridOrigin[1] + verts[:,1]
    mesh_points[:,2] = voxelGridOrigin[2] + verts[:,2]

    # permute faces to get visualization
    # faces = np.flip(faces, 1)


    # try writing to the ply file
    print "converting numpy arrays to format for ply file"
    ply_conversion_start_time = time.time()

    num_verts = verts.shape[0]
    num_faces = faces.shape[0]

    verts_tuple = np.zeros((num_verts,), dtype=[('x', 'f4'), ('y', 'f4'),
                                                ('z', 'f4')])
    faces_tuple = np.zeros((num_faces,), dtype=[('vertex_indices', 'i4', (3,))])

    for i in xrange(0, num_verts):
        verts_tuple[i] = tuple(mesh_points[i, :])

    for i in xrange(0, num_faces):
        faces_tuple[i] = faces[i, :].tolist()



    # save it out
    # try to save it
    el_verts = PlyElement.describe(verts_tuple, 'vertex')
    el_faces = PlyElement.describe(faces_tuple, 'face')

    ply_data = PlyData([el_verts, el_faces])
    print "saving mesh to %s" %(tsdf_mesh_filename)
    ply = ply_data.write(tsdf_mesh_filename)

    print "converting to ply format and writing to file took", time.time() - start_time
