import rospy
import std_msgs.msg
import visualization_msgs.msg

class GripperParams:

    @staticmethod
    def make_schunk_params():
        params = dict()
        params['hand_inner_diameter'] = 0.08
        params['finger_width'] = 0.018
        params['hand_depth'] = 0.06
        params['hand_height'] = 0.023
        # params['palm_depth'] = 0.075 # this is accurate
        params['palm_depth'] = 0.02 # better for vis
        params['init_bite'] = 0.02
        return params


class Gripper(object):

    def __init__(self, params):
        """
        :param params: dict of parameters of the type produced by GripperParams class
        """
        self._params = params

    @property
    def params(self):
        return self._params

    @staticmethod
    def make_header(frame_id, stamp):
        header = std_msgs.msg.Header()
        header.frame_id = frame_id
        header.stamp = stamp
        return header

    def make_rviz_visualization_msg(self, frame_id, stamp):
        """

        :param frame_id: string
        :param stamp: ros.Time object
        :return: visualization_msgs/MarkerArray
        """

        marker_array = visualization_msgs.msg.MarkerArray()
        marker_array.markers.append(self.make_right_finger_message(frame_id, stamp))
        marker_array.markers.append(self.make_left_finger_message(frame_id, stamp))
        marker_array.markers.append(self.make_palm_message(frame_id, stamp))
        # marker_array.markers.append(self.make_camera_housing_message(frame_id, stamp))


        return marker_array

    def make_right_finger_message(self, frame_id, stamp):
        """

        :param frame_id: string
        :param stamp: ros.Time object
        :return: visualization_msgs/Marker
        """
        marker = visualization_msgs.msg.Marker()
        marker.ns = "gripper"
        marker.id = 0
        marker.type = marker.CUBE
        marker.header = Gripper.make_header(frame_id, stamp)
        marker.pose.position.x = self.params['hand_depth']/2.0
        marker.pose.position.y = -(self.params['hand_inner_diameter'] + self.params['finger_width'])/2.0
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.params['hand_depth']
        marker.scale.y = self.params['finger_width']
        marker.scale.z = self.params['hand_height']

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker

    def make_left_finger_message(self, frame_id, stamp):
        """

        :param frame_id: string
        :param stamp: ros.Time object
        :return: visualization_msgs/Marker
        """

        marker = visualization_msgs.msg.Marker()
        marker.ns = "gripper"
        marker.id = 1
        marker.type = marker.CUBE
        marker.header = Gripper.make_header(frame_id, stamp)
        marker.pose.position.x = self.params['hand_depth'] / 2.0
        marker.pose.position.y = (self.params['hand_inner_diameter'] + self.params['finger_width']) / 2.0
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.params['hand_depth']
        marker.scale.y = self.params['finger_width']
        marker.scale.z = self.params['hand_height']

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker

    def make_palm_message(self, frame_id, stamp):
        """

        :param frame_id: string
        :param stamp: ros.Time object
        :return: visualization_msgs/Marker
        """
        marker = visualization_msgs.msg.Marker()
        marker.ns = "gripper"
        marker.id = 2
        marker.type = marker.CUBE
        marker.header = Gripper.make_header(frame_id, stamp)
        marker.pose.position.x = -self.params['palm_depth']/2.0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.params['palm_depth']
        marker.scale.y = self.params['hand_inner_diameter'] + 2*self.params['finger_width']
        marker.scale.z = self.params['hand_height']

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker

    def make_camera_housing_message(self, frame_id, stamp):
        """

        Depth in Z direction is 7.5
        Starts at -palm_depth/2.0
        :param frame_id: string
        :param stamp: ros.Time object
        :return: visualization_msgs/Marker
        """

        camera_height = 0.075
        marker = visualization_msgs.msg.Marker()
        marker.ns = "gripper"
        marker.id = 3
        marker.type = marker.CUBE
        marker.header = Gripper.make_header(frame_id, stamp)
        marker.pose.position.x = -self.params['palm_depth'] / 2.0
        marker.pose.position.y = 0
        marker.pose.position.z = -(self.params['hand_height'] + camera_height) / 2.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.params['palm_depth']
        marker.scale.y = self.params['hand_inner_diameter'] + 2 * self.params['finger_width']
        marker.scale.z = camera_height

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker

    @staticmethod
    def make_schunk_gripper():
        return Gripper(GripperParams.make_schunk_params())




