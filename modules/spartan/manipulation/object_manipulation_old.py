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

    def __init__(self, poser_visualizer=None, grasp_data=None):
        self._poser_visualizer = poser_visualizer

        self._grasp_data = grasp_data
        self._T_obs_model = None
        self._T_W_model_target = None

        self._debug_vis_container = om.getOrCreateContainer("Debug Object Manip")

    def _setup_config(self):
        self._config = dict()
        self._config['object_name'] = 'shoe'


        pos = np.array([0.06883578, -0.01160232, -0.00817785])
        quat = np.array([9.99806766e-01, -1.43918037e-02, -1.33767536e-02, -6.07840632e-04])
        self._config['T_W_model_target_default'] = transformUtils.transformFromPose(pos, quat)

        pos = np.array([0.74018759, 0.01560964, 0.09624007])
        quat = np.array([0.70365365, -0.16242853, 0.68819834, 0.06979651])
        self._config['T_W_G_default'] = transformUtils.transformFromPose(pos, quat)

        self.T_W_model_target = self._config['T_W_model_target_default']

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

    @property
    def T_obs_model(self):
        """
        A vtkTransform

        Rigid transform from model to the observation.
        This usually comes from the output of poser
        :return:
        """
        return self._T_obs_model

    @T_obs_model.setter
    def T_obs_model(self, value):
        self._T_obs_model = value

    @property
    def T_W_model_target(self):
        """
        Desired location of model in world. This specifies the goal
        :return: vtkTransform
        """

        return self._T_W_model_target

    @T_W_model_target.setter
    def T_W_model_target(self, value):
        """
        :param value: vtkTransform
        :return:
        """
        self._T_W_model_target = value

    def setup(self):
        """
        Populates the transforms
        :return:
        """
        self._T_obs_model = self._poser_visualizer.get_model_to_object_transform(self._config['object_name'])

    def compute_transforms(self):
        """
        Make sure that grasp_data is set before you call this function
        :return:
        """

        # gripper palm to world
        T_W_G = transformUtils.copyFrame(self.grasp_data.grasp_frame)

        # compute desired transform of observation points
        T_model_obs = self.T_obs_model.GetLinearInverse()
        T_W_obs_desired = transformUtils.concatenateTransforms([T_model_obs, self._T_W_model_target])

        # compute desired gripper location
        T_W_G = self.grasp_data.grasp_frame
        T_model_W = T_model_obs
        T_model_G = transformUtils.concatenateTransforms([T_W_G, T_model_W])

        # Gn = grasp next/new
        T_G_Gn = transformUtils.concatenateTransforms([T_model_G, self._T_W_model_target])
        T_W_Gn = transformUtils.concatenateTransforms([T_G_Gn, T_W_G])


        # record the results
        self._T_W_Gn = T_G_Gn
        self._T_W_obs_desired = T_W_obs_desired


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
        template_poly_data = self._poser_visualizer.get_template_poly_data(self._config['object_name'])
        vis_dict['template'] = vis.updatePolyData(template_poly_data, 'template goal', parent=self._vis_container,
                                                  color=[0, 1, 0])
        vis_dict['template'].actor.SetUserTransform(self.T_W_model_target)


        # observation
        obs_poly_data = self._poser_visualizer.get_observation_poly_data(self._config['object_name'])
        vis_dict['observation'] = vis.updatePolyData(obs_poly_data, 'observation goal', parent=self._vis_container,
                                                  color=[1, 0, 0])
        vis_dict['observation'].actor.SetUserTransform(self._T_W_obs_desired)

        vis.updateFrame(self.T_W_Gn, 'Place Grasp Frame', scale=0.15, parent=self._vis_container)


    def extract_data_from_poser_visualizer(self):
        """
        Assign the relevant fields from poser visualizer

        :return:
        """
        self.T_obs_model = self._poser_visualizer.get_model_to_object_transform(self._config['object_name'])

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
        Assign some default values
        :return:
        """
        self.T_W_G = self._config['T_W_G_default']
        self.T_W_model_target = self._config['T_W_model_target_default']
        grasp_data = GraspData(self.T_W_G)
        grasp_data.gripper = Gripper.make_schunk_gripper()
        self.grasp_data = grasp_data

        self.extract_data_from_poser_visualizer()


    def test(self):
        """
        Test routine
        :return:
        """
        self._poser_visualizer.visualize_result()
        self.assign_defaults()
        self.compute_transforms()
        self.visualize()

    @staticmethod
    def load_mug_model():
        """
        Loads a mug model
        :return:
        :rtype:
        """

        mug_file = os.path.join(spartan_utils.get_data_dir(), "pdc/templates/mugs/mug.stl")
        poly_data = ioUtils.readPolyData(mug_file)
        return poly_data

    def load_and_visualize_mug_model(self):
        """
        Loads and visualizes a mug model
        :return:
        :rtype:
        """
        poly_data = ObjectManipulation.load_mug_model()
        # t = vtk.vtkTransform()
        # t.Translate(0,0,-0.025)
        # poly_data = filterUtils.transformPolyData(poly_data, t)
        self.model = vis.updatePolyData(poly_data, "mug", parent=self._debug_vis_container)
        vis.addChildFrame(self.model)

        # Transform from model to new model
        self._T_Mn_M = ObjectManipulation.get_affine_transform_for_mug()
        poly_data_transformed = filterUtils.transformPolyData(poly_data, self._T_Mn_M)
        self.transformed_model = vis.updatePolyData(poly_data_transformed, 'mug affine', parent=self._debug_vis_container)

    def save_poly_data(self):
        """
        Saves the poly data to a file
        :return:
        :rtype:
        """
        mug_file = os.path.join(spartan_utils.get_data_dir(), "pdc/templates/mugs/mug.stl")
        ioUtils.writePolyData(self.model.polyData, mug_file)


    @staticmethod
    def get_affine_transform_for_mug():
        t = vtk.vtkTransform()
        s_x = 1.5
        s_y = 1.5
        s_z = 1.5
        t.Scale(s_x, s_y, s_z)
        return t








