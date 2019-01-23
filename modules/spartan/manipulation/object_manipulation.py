# system
import numpy as np
import os

# director
from director import transformUtils
import director.objectmodel as om
import director.visualization as vis
from director import ioUtils
from director import vtkAll as vtk
from director import filterUtils

# spartan
from spartan.manipulation.gripper import Gripper
from spartan.manipulation.grasp_data import GraspData
import spartan.utils.utils as spartan_utils

class ObjectManipulation(object):
    """
    Class to facilitate picking and placing objects.

    Specifically for interfacing with pdc and poser
    """

    def __init__(self, T_goal_object=None, grasp_data=None, T_W_G=None):
        """

        :param T_goal_object: vtkTransform
        :param grasp_data:
        """

        self._T_goal_object = T_goal_object
        self._grasp_data = grasp_data
        self.T_W_G = T_W_G

    @property
    def grasp_data(self):
        """
        Should by of type spartan.manipulation.grasp_data.GraspData
        :return:
        """
        return self._grasp_data

    @grasp_data.setter
    def grasp_data(self, value):
        self._grasp_data = value

    @property
    def T_W_G(self):
        """
        Grasp to world transform
        :return: vtkTransform
        """
        return self._T_W_G

    @T_W_G.setter
    def T_W_G(self, value):
        """

        :param value: vtkTransform
        :return:
        """
        self._T_W_G = value

    @property
    def T_W_Gn(self):
        """
        The frame at which the gripper should be when placing the object
        :return:
        """
        return self._T_W_Gn

    def compute_transforms(self):
        """
        Make sure that grasp_data is set before you call this function
        :return:
        """

        # gripper palm to world
        # this also happens to be gripper palm to object
        if self.T_W_G is None:
            self.T_W_G = transformUtils.copyFrame(self.grasp_data.grasp_frame)

        T_W_Gn = transformUtils.concatenateTransforms([self.T_W_G, self._T_goal_object])


        self._T_W_Gn = T_W_Gn


    def _clear_visualization(self):
        """
        Delete the 'object manipulation' vis container, create a new one with the same name
        :return:
        :rtype:
        """
        self._vis_container = om.getOrCreateContainer("object_manipulation")
        om.removeFromObjectModel(self._vis_container)
        self._vis_container = om.getOrCreateContainer("object_manipulation")
        self._vis_dict = dict()

    def visualize(self):
        """
        Visualize
        :return:
        """
        self._clear_visualization()

        # model
        vis_dict = self._vis_dict
        # template_poly_data = self._poser_visualizer.get_template_poly_data(self._config['object_name'])
        # vis_dict['template'] = vis.updatePolyData(template_poly_data, 'template goal', parent=self._vis_container,
        #                                           color=[0, 1, 0])
        # vis_dict['template'].actor.SetUserTransform(self.T_W_model_target)
        #
        #
        # # observation
        # obs_poly_data = self._poser_visualizer.get_observation_poly_data(self._config['object_name'])
        # vis_dict['observation'] = vis.updatePolyData(obs_poly_data, 'observation goal', parent=self._vis_container,
        #                                           color=[1, 0, 0])
        # vis_dict['observation'].actor.SetUserTransform(self._T_W_obs_desired)


        vis.updateFrame(self.T_W_G, "Initial Grasp Frame", scale=0.15, parent=self._vis_container)
        vis.updateFrame(self.T_W_Gn, 'Place Gripper Frame', scale=0.15, parent=self._vis_container)

    def get_place_grasp_data(self):
        """
        Get GraspData for the place action
        :return:
        """
        gd = GraspData(self.T_W_Gn)
        gd.gripper = self.grasp_data.gripper.copy()
        return gd

    def assign_defaults(self):
        """
        Assign some default values for testing
        :return:
        """

        # T_goal_object
        pos = np.array([1.05195929e+00, -4.72744658e-01, -8.71958505e-04])
        quat = np.array([0.38988508, -0.0049511, -0.0066051, 0.92082646])

        T_goal_obs = transformUtils.transformFromPose(pos,quat)
        self._T_goal_object = T_goal_obs

        # T_W_G
        pos = np.array([0.74018759, 0.01560964, 0.09624007])
        quat = np.array([0.70365365, -0.16242853, 0.68819834, 0.06979651])
        self.T_W_G = transformUtils.transformFromPose(pos, quat)

        grasp_data = GraspData(self.T_W_G)
        grasp_data.gripper = Gripper.make_schunk_gripper()
        self.grasp_data = grasp_data


    def test(self):
        """
        Test routine
        :return:
        """
        self.assign_defaults()
        self.compute_transforms()
        self.visualize()







