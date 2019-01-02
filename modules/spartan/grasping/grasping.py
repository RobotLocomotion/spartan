import os
import numpy as np
import torch
import math

import spartan.utils.utils as spartan_utils
from spartan.perception.heightmap import HeightMap

class HeightmapGraspPlanner(object):

    def __init__(self, config):
        self._config = config
        self._default_heightmap = HeightMap.make_default()
        self._dx = self._default_heightmap.dx
        self._dy = self._default_heightmap.dy
        self._setup()


    def plan_grasp_on_heightmap(self, heightmap_obj):
        # steps
        # 1. generate grasp candidates
        # 2. score grasp candidates


    def setup(self):
        # patch is 10cm in size
        patch_size = int(math.ceil(0.1/min(self._dx, self._dy)))

        # make sure patch size is odd
        if patch_size % 2 == 0:
            patch_size += 1

        self._patch_size = patch_size
        self._patch_center = (patch_size-1) / 2 + 1 # since python indices start with zero



        self._masks = dict()

        for width in self._config['sampling']['gripper_width']:
            d = dict()
            d['left_finger'] =

            # left finger
            between_fingers = np.zeros([self._patch_size, self._patch_size], dtype=int)
            x_idx = int(math.floor(width/(2.0 * self._dx)))
            xmin = self._patch_center - x_idx
            xmax = self._patch_center + x_idx
            ymin = self._patch_center - y_idx
            ymax = self._patch_center + y_idx
            y_idx = int(math.floor(self._config['gripper']['finger_height']/(2.0 * self._dy)))

            between_fingers[self._patch_center-x_idx:self._patch_center + x_idx,
            self._patch_center-y_idx:self._patch_center+y_idx] = 1

            d['between_fingers'] = between_fingers


            d['left_finger'] =

        self._masks[width] = d

        # left finger collsion
        left_finger_collision



    def score_grasps_on_heightmap(self, heightmap, dx, dy):
        """
        Score every single grasp in the heightmap
        Grasps are already assumed to be with x being the closing
        direction
        :param heightmap: numpy array [N,M]
        :type heightmap:
        :param dx: dx direction
        :type dx: dy direction
        :param dy:
        :type dy:
        :return:
        :rtype:
        """

        # put things on the GPU
        h = torch.from_numpy(heightmap)





    def sample_candidate_points(self, heightmap_obj):
        # threshold only for grasps a certain distance above
        hm = heightmap_obj.heightmap

        idx = np.where(hm > self._config['sampling']['min_height_above_table'])

        # do non-max suppression here for spatial separation

    @staticmethod
    def make_default():


        config_file = os.path.join(spartan_utils.getSpartanSourceDir(), 'src', 'catkin_projects',
                     'station_config', 'RLG_iiwa_1', 'manipulation', 'heightmap_grasping.yaml')

        config = spartan_utils.getDictFromYamlFilename(config_file)

        return HeightmapGraspPlanner(config)


    @staticmethod
    def make_mask(shape, xmin, xmax, ymin, ymax):

