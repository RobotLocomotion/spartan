# system
import numpy as np
import copy

# director
from director import transformUtils

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils
import spartan.utils.director_utils as director_utils
import spartan.utils.control_utils as control_utils
import spartan.manipulation.gripper

class GraspData(object):
    """
    Class to store some grasp data

    Mapping from camera optical frame to gripper frame. The axes
    are aligned, they just need to be rotated

    camera --> gripper
    x          y
    y          z
    z          x

    # frame names
    T_W_G: grasp to world
    T_W_PG: pre-grasp to world

    """

    # gripper frame (G) to gripper frame camera axes (GC)
    T_GC_G = transformUtils.getTransformFromAxes([0, 0, 1], [1, 0, 0], [0, 1, 0])

    def __init__(self, T_W_G=None):
        """
        T_W_G transform from gripper frame (palm) to world
        :param T_W_G:
        """
        # Palm Frame as shown in this README
        # https://github.com/RobotLocomotion/spartan/blob/master/doc/grasping.md
        self._T_W_G = T_W_G
        self._T_W_PG = None
        self._gripper_width = None # only used for ggcnn
        self._type = None  # either ggcnn or spartan_grasp
        self._data = dict()  # random additional info you might want to store
        self._gripper = None # gripper object

    def compute_pre_grasp_frame(self, distance=0.15):
        """
        Computes the pre-grasp frame gotten by pushing this grasp back
        along the x-direction by distance (in meters).
        :param distance: distasnce to push back frame along the x-direction
        :return: vtkTransform. Also stores this in self._pre_grasp_frame
        """
        pos = [-1.0 * distance, 0, 0]
        quat = [1, 0, 0, 0]
        T_G_PG = transformUtils.transformFromPose(pos, quat)

        # T_W_PG = T_W_G * T_G_PG
        self._T_W_PG = transformUtils.concatenateTransforms([T_G_PG, self._T_W_G])

        return self._T_W_PG

    @staticmethod
    def from_ggcnn_grasp_frame_camera_axes(T_W_GC):
        """

        :param T_W_GC: vtkTransform. grasp (in camera axes convention) to world transform
        :return:
        """
        T_W_G = transformUtils.concatenateTransforms([GraspData.T_GC_G, T_W_GC])
        return GraspData(T_W_G)

    @property
    def data(self):
        return self._data

    @property
    def grasp_frame(self):
        return self._T_W_G

    @property
    def pre_grasp_frame(self):
        return self._T_W_PG

    @property
    def gripper(self):
        return self._gripper

    @gripper.setter
    def gripper(self, value):
        self._gripper = value

    @property
    def grasp_inner_diameter(self):
        """
        The distance between the inner faces of the fingers (in meters)
        If not specified returns None
        :return: float or None
        """

        gripper_width = None
        if self.gripper is not None:
            gripper_width = self.gripper.params['hand_inner_diameter']

        if self._gripper_width is not None:
            gripper_width = self._gripper_width

        return gripper_width

    def rotate_grasp_frame_to_nominal(self, grasp_z_axis_nominal):
        self._T_W_G = GraspData.rotate_grasp_frame_to_nominal_dir(self._T_W_G, grasp_z_axis_nominal)

        self.compute_pre_grasp_frame()

    @staticmethod
    def rotate_grasp_frame_to_nominal_dir(grasp_frame, grasp_z_axis_nominal):
        """
        Rotates the grasp frame around it's x-axis so that the grasp_z_axis is
        most aligned with the grasp_z_axis_nominal

        For normal picking in front of robot choose grasp_z_axis_nominal = [1,0,0]

        :param: grasp_frame vtkTransform
        :param: grasp_z_axis_nominal numpy vector of size 3
        """

        grasp_frame_z_axis = grasp_frame.TransformVector(0, 0, 1)
        if (np.dot(grasp_frame_z_axis, grasp_z_axis_nominal) < 0):
            grasp_frame.PreMultiply()
            grasp_frame.RotateX(180)

        return grasp_frame

    @staticmethod
    def from_spartan_grasp(msg):
        """
        Create grasp object from grasp message
        :param msg: spartan_grasp_msgs/Grasp
        :return:
        """

        T_W_grasp_palm = spartanUtils.transformFromROSPoseMsg(msg.pose.pose)
        grasp_data = GraspData(T_W_grasp_palm)
        grasp_data.data['msg'] = msg
        grasp_data.data['planner'] = 'spartan_grasp'

        # make gripper object
        grasp_data._gripper = spartan.manipulation.gripper.Gripper.from_spartan_grasp_params_msg(msg.params)

        return grasp_data

    def copy(self):
        """
        Returns a copy
        :return:
        """
        grasp_data = GraspData(self.grasp_frame)
        grasp_data.gripper = copy.deepcopy(self.gripper)

        return grasp_data