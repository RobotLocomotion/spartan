# system
import os
import yaml
from yaml import CLoader
import numpy as np

import director.objectmodel as om
import director.visualization as vis
from director import ioUtils
from director import transformUtils
import director.vtkNumpy as vnp



class PoserVisualizer(object):

    def __init__(self, poser_output_folder = None):
        self._clear_visualization()
        self._poser_output_folder = poser_output_folder

    @property
    def poser_output_folder(self):
        """
        The full path to the poser output folder
        :return:
        :rtype:
        """
        return self._poser_output_folder

    @poser_output_folder.setter
    def poser_output_folder(self, value):
        self._poser_output_folder = value

    def load_poser_response(self):
        """
        Load the poser_response.yaml file
        :return:
        :rtype: dict
        """

        filename = self._convert_relative_path_to_absolute("poser_response.yaml")
        return yaml.load(file(filename), Loader=CLoader)


    def _convert_relative_path_to_absolute(self, path):
        """
        Converts a path that is relative to self.poser_output_folder to an
        absolute path.

        You must ensure that self.poser_output_folder is not
        None before calling this function
        :param path:
        :type path:
        :return:
        :rtype:
        """
        if self._poser_output_folder is None:
            raise ValueError("poser_output_folder cannot be None")

        return os.path.join(self._poser_output_folder, path)

    def _clear_visualization(self):
        """
        Delete the Poser vis container, create a new one with the same name
        :return:
        :rtype:
        """
        self._poser_vis_container = om.getOrCreateContainer("Poser")
        om.removeFromObjectModel(self._poser_vis_container)
        self._poser_vis_container = om.getOrCreateContainer("Poser")


    def visualize_result(self, poser_response=None):
        """
        Visualizes the results of running poser

        :param poser_response:
        :type poser_response: dict loaded from poser_response.yaml file
        :return:
        :rtype:
        """

        if poser_response is None:
            poser_response = self.load_poser_response()

        self._clear_visualization()
        self._object_vis_containers = dict()

        # visualize the observation
        for object_name, data in poser_response.iteritems():
            vis_dict = dict()
            self._object_vis_containers[object_name] = vis_dict
            vis_dict['container'] = om.getOrCreateContainer(object_name,
                                                            parentObj=self._poser_vis_container)

            # transform from template to observation
            T_obs_template = PoserVisualizer.parse_transform(data['rigid_transform'])



            # usually a pcd
            template_file = self._convert_relative_path_to_absolute(data['image_1']['save_template'])
            template_poly_data = PoserVisualizer.readPolyData(template_file)
            vis_dict['template'] = vis.updatePolyData(template_poly_data, 'template', parent=vis_dict['container'],
                               color=[0,1,0])

            vis_dict['template'].actor.SetUserTransform(T_obs_template)

            # transform from template to observation
            T_nonrigid_obs_template = PoserVisualizer.parse_transform(data['affine_transform'])
            template_file = self._convert_relative_path_to_absolute(data['image_1']['save_template'])
            template_poly_data = PoserVisualizer.readPolyData(template_file)
            vis_dict['template_affine'] = vis.updatePolyData(template_poly_data, 'template affine', parent=vis_dict['container'],
                                                      color=[1, 0, 1])


            vis_dict['template_affine'].actor.SetUserTransform(T_nonrigid_obs_template)




            # usually a pcd
            observation_file = self._convert_relative_path_to_absolute(data['image_1']['save_processed_cloud'])

            observation_poly_data = PoserVisualizer.readPolyData(observation_file)
            vis_dict['observation'] = vis.updatePolyData(observation_poly_data, 'observation',
                                                       parent=vis_dict['container'])



    def get_model_to_object_transform(self, object_name):
        """
        Returns the transform from model to object for the given object

        :param object_name: str
        :return: vtkTransform
        """

        T_obs_model = self._object_vis_containers[object_name]['template'].actor.GetUserTransform()
        return transformUtils.copyFrame(T_obs_model)

    def get_template_poly_data(self, object_name):
        """
        Returns the poly data for the template of the given object
        :param object_name: str
        :return: vtkPolyData
        """
        return self._object_vis_containers[object_name]['template'].polyData

    def get_observation_poly_data(self, object_name):
        """
        Returns the poly data for the given object
        :param object_name: str
        :return: vtkPolyData
        """
        return self._object_vis_containers[object_name]['observation'].polyData


    def test(self):
        import dense_correspondence_manipulation.utils.director_utils as director_utils
        polyData = director_utils.readPlyFile("/home/manuelli/sandbox/poser/template.ply")
        vis.showPolyData(polyData, 'test')

    @staticmethod
    def parse_transform(transform_matrix_list):
        """
        Returns a vtkTransform matrix from column major list of matrix coefficients
        :param transform_matrix_list:
        :type transform_matrix_list:
        :return:
        :rtype:
        """

        matrix_coeffs = np.array(transform_matrix_list) # vector of length 16
        mat = np.reshape(matrix_coeffs, [4,4], order='F')
        return transformUtils.getTransformFromNumpy(mat)

    @staticmethod
    def make_default_spartan():
        """
        Makes poser
        :return:
        """
        # spartan
        import spartan.utils.utils as spartanUtils

        path_to_poser_output = os.path.join(spartanUtils.get_sandbox_dir(), "poser")

        if not os.path.exists(path_to_poser_output):
            raise ValueError("poser output folder %s doesn't exist" %(path_to_poser_output))

        return PoserVisualizer(path_to_poser_output)

    @staticmethod
    def make_default():
        """
        Makes poser
        :return:
        """
        # spartan


        path_to_poser_output = os.getenv("POSER_OUTPUT_DIR")
        if path_to_poser_output is None:
            path_to_poser_output = os.path.join(os.path.expanduser("~"), "sandbox", "poser")

        # if not os.path.exists(path_to_poser_output):
        #     raise ValueError("poser output folder %s doesn't exist" % (path_to_poser_output))

        return PoserVisualizer(path_to_poser_output)

    @staticmethod
    def readPolyData(file):
        """
        Read poly data while correctly dispatching to special
        handler for ply files
        """
        extension = os.path.splitext(file)[1]

        if extension == ".ply":

            from plyfile import PlyData

            plydata = PlyData.read(file)
            vertex_data = plydata['vertex'].data  # numpy array with fields ['x', 'y', 'z']
            pts = np.zeros([vertex_data.size, 3])
            pts[:, 0] = vertex_data['x']
            pts[:, 1] = vertex_data['y']
            pts[:, 2] = vertex_data['z']

            return vnp.numpyToPolyData(pts)

        else:
            return ioUtils.readPolyData(file)