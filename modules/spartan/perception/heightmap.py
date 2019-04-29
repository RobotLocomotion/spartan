# system
import numpy as np
import math

# spartan
import spartan.perception.utils as perception_utils

class HeightMap(object):

    def __init__(self, xdim, ydim, zdim, N, M, P):
        self._xdim = xdim
        self._ydim = ydim
        self._zdim = zdim


        self._N = N
        self._M = M
        self._P = P

        self._initialize()

    def _initialize(self):
        """
        Sets up some initial data structures
        :return:
        :rtype:
        """

        self._dx = (self._xdim[1] - self._xdim[0]) / (1.0 * self._N)
        self._dy = (self._ydim[1] - self._ydim[0]) / (1.0 * self._M)
        self._dz = (self._zdim[1] - self._zdim[0]) / (1.0 * self._P)

        self._min_coords = np.array([self._xdim[0], self._ydim[0], self._zdim[0]])
        self._deltas = np.array([self._dx, self._dy, self._dz])
        self._num_bins = np.array([self._N, self._M, self._P])

        self._occupancy_grid = np.zeros([self._N, self._M, self._P], dtype=bool)
        self._z_height_vec = self._dz * np.arange(0, self._P)
        self._recompute_heightmap()

        self._xy_grid = np.zeros([self._N, self._M, 2])
        self._xy_grid[:,:,0] = self._xdim[0] + self._dx * np.repeat(np.arange(0, self._N).reshape((self._N, 1)), self._M, axis=1)
        self._xy_grid[:, :, 1] = self._ydim[0] + self._dy * np.repeat(np.arange(0, self._M).reshape((1,self._M)), self._N, axis=0)


    @property
    def heightmap(self):
        return self._heightmap

    @property
    def xy_grid(self):
        return self._xy_grid

    @property
    def dx(self):
        return self._dx

    @property
    def dy(self):
        return self._dy

    def _recompute_heightmap(self):
        """
        Updates the heightmap given the occupancy grid
        :return:
        :rtype:
        """
        occupancy_height = self._occupancy_grid * self._z_height_vec[None, None, :]

        # numpy array of float with shape [N,M]
        self._heightmap = np.max(occupancy_height, axis=2)

    def insert_pointcloud_into_heightmap(self, pc, T_H_pc=None):
        """

        :param T_H_pc: 4 x 4 homogeneous transform from pointcloud to
        heightmap frame
        :type T_H_pc:
        :param pc: numpy with shape [K,3] where K is num points
        :type pc:
        :return:
        :rtype:
        """

        if T_H_pc is not None:
            pc = perception_utils.transform_pointcloud(pc, T_H_pc)

        self._pc = pc

        # convert to indices
        # K x 3, but the 3 are in the range [0,N], [0,M], [0,P]
        pc_idx = (pc - self._min_coords[None, :])/self._deltas[None,:]

        self._pc_idx = pc_idx

        # clip any indexes that are out of the range
        # boolean array with shape [K,3]
        idx_valid_matrix = np.logical_and((pc_idx < (self._num_bins - 1)[None, :]), pc_idx > 0)

        self._idx_valid_matrix = idx_valid_matrix

        # all indices in a row need to be valid
        # vector with shape (K,)
        idx_valid = np.all(idx_valid_matrix, axis=1)
        self._idx_valid = idx_valid

        # print "idx_valid.size", idx_valid.size

        # (K', 3), dtype=int
        pc_idx_valid = np.floor(pc_idx[idx_valid, :]).astype(int)

        self._pc_idx_valid = pc_idx_valid

        # insert it into the heightmap
        # print "np.count_nonzero(self._occupancy_grid)", np.count_nonzero(self._occupancy_grid)
        self._occupancy_grid[pc_idx_valid[:,0], pc_idx_valid[:,1], pc_idx_valid[:,2]] = True
        # print "np.count_nonzero(self._occupancy_grid) after", np.count_nonzero(self._occupancy_grid)
        self._recompute_heightmap()

    # convert heightmap to pointcloud
    def heightmap_to_pointcloud(self):
        """
        Return numpy record array with shape [K,3] and fields 'x', 'y', 'z'
        :return:
        :rtype:
        """
        # only add ones where the height is positive
        z_height = self._zdim[0] + self._heightmap
        z_height = z_height[self._heightmap > 0]
        xy_coords = self._xy_grid[self._heightmap > 0]

        K = xy_coords.shape[0]
        pc = np.concatenate((xy_coords, z_height.reshape([K,1])), axis=1)
        pc_rec_array = np.recarray(K, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])

        pc_rec_array['x'] = pc[:,0]
        pc_rec_array['y'] = pc[:, 1]
        pc_rec_array['z'] = pc[:, 2]

        return pc_rec_array


    @staticmethod
    def make_default():
        voxel_size = 0.0025
        xdim = [0.4, 0.88]
        ydim = [-0.25, 0.38]
        zdim = [-0.01, 0.4]

        N = int(math.ceil((xdim[1] - xdim[0]) / voxel_size))
        M = int(math.ceil((ydim[1] - ydim[0]) / voxel_size))
        P = int(math.ceil((zdim[1] - zdim[0]) / voxel_size))
        hm = HeightMap(xdim, ydim, zdim, M, N, P)
        return hm







