import numpy as np

def transform_pointcloud(pc, T):
    """
    Applies the homogeneous transform T to the pointcloud
    :param pc: numpy array [N,3]
    :type pc:
    :param T: 4 x 4 homogeneous transform
    :type T:
    :return:
    :rtype:
    """

    N = np.shape(pc)[0]
    pc_homog = np.concatenate((pc, np.ones([N,1])), axis=1) # N x 4

    pc_transformed = np.matmul(T, pc_homog.transpose()) # 4 x N
    pc_transformed = pc_transformed.transpose() # N x 4
    pc_transformed = pc_transformed[:, :3] # N x 3

    return pc_transformed
