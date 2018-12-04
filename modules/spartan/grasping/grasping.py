import os
import numpy as np


import spartan.utils.utils as spartan_utils


class HeightmapGraspPlanner(object):

    def __init__(self, config):
        self._config = config



    def plan_grasp_on_heightmap(self, heightmap_obj):
        # steps
        # 1. generate grasp candidates
        # 2. score grasp candidates


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


