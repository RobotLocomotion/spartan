# system
import numpy as np
import os

# ROS
import rospy
import ros_numpy
import tf2_ros
import sensor_msgs

# director
import director.objectmodel as om
import director.vtkNumpy as vnp
import director.transformUtils as transformUtils
import director.filterUtils as filterUtils
import director.visualization as vis
from director.timercallback import TimerCallback
import director.ioUtils as ioUtils

# this one works better than director version for some reason
from spartan.utils.taskrunner import TaskRunner
import spartan.utils.ros_utils as ros_utils
import spartan.utils.utils as spartan_utils

class DirectorROSVisualizer(object):

    def __init__(self, tf_buffer=None):
        self.taskRunner = TaskRunner()
        self._tf_buffer = tf_buffer
        self.clear_visualization()
        self._subscribers = dict()
        self._expressed_in_frame = "base"

    def setup_TF(self):
        """
        Sets up TF
        :return:
        :rtype:
        """
        if self._tf_buffer is None:
            self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def clear_visualization(self):
        """
        Clears the visualization
        :return:
        :rtype:
        """
        container_name = "ROS"
        self._vis_container = om.getOrCreateContainer(container_name)
        om.removeFromObjectModel(self._vis_container)
        self._vis_container = om.getOrCreateContainer(container_name)

    def add_subscriber(self, topic, name=None, call_in_thread=True, visualize=False, msg_type=None):
        """
        Adds a subscriber
        :param topic:
        :type topic:
        :param call_in_thread:
        :type call_in_thread:
        :return:
        :rtype:
        """


        if call_in_thread:
            self.add_subscriber(topic, name=name, call_in_thread=False, visualize=visualize, msg_type=msg_type)
            return

        if msg_type is None:
            msg_type = sensor_msgs.msg.PointCloud2

        if name is None:
            name = topic


        subscriber = ros_utils.SimpleSubscriber(topic, msg_type)
        subscriber.start()

        d = dict()
        d['subscriber'] = subscriber
        d['topic'] = topic
        d['visualize'] = visualize
        d['name'] = name

        self._subscribers[topic] = d

    def update(self, snapshot=False):
        """
        Visualizes the pointclouds set to true. This should be called from the main thread
        :return:
        :rtype:
        """

        for topic, data in self._subscribers.iteritems():

            if not data['visualize']:
                continue

            msg = data['subscriber'].lastMsg
            if msg is None:
                continue

            # get frame
            # TransformStamped
            # this might need to be called in thread
            try:
                T_W_pointcloud_stamped = self._tf_buffer.lookup_transform(self._expressed_in_frame, msg.header.frame_id, msg.header.stamp)
            except:
                continue


            T_W_pointcloud = ros_numpy.numpify(T_W_pointcloud_stamped.transform)
            T_W_pointcloud_vtk = transformUtils.getTransformFromNumpy(T_W_pointcloud)
            pointcloud_numpy = DirectorROSVisualizer.numpy_from_pointcloud2_msg(msg)
            pointcloud_vtk = vnp.getVtkPolyDataFromNumpyPoints(pointcloud_numpy)
            pointcloud_vtk = filterUtils.transformPolyData(pointcloud_vtk, T_W_pointcloud_vtk)

            data['pointcloud'] = pointcloud_vtk
            if snapshot:
                name = data["name"] + " snapshot"
                vis.showPolyData(pointcloud_vtk, name, parent=self._vis_container)
            else:
                vis.updatePolyData(pointcloud_vtk, data['name'], parent=self._vis_container)

    def start(self, rate_hz=30):
        """
        Launches a timercallback that just calls update()
        :param rate_hz:
        :type rate_hz:
        :return:
        :rtype:
        """
        self._timercallback = TimerCallback(targetFps=rate_hz, callback=self.update)
        self._timercallback.start()

    def stop(self):
        """
        Stops the visualization callback
        :return:
        :rtype:
        """
        self._timercallback.stop()

    def snapshot(self):
        self.update(snapshot=True)

    def save_snapshot(self):
        save_dir = os.path.join(spartan_utils.get_sandbox_dir(), "pointclouds")
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        for topic, data in self._subscribers.iteritems():
            filename = os.path.join(save_dir, "%s.ply" %(data['name']))
            filename.replace(" ", "_")
            ioUtils.writePolyData(data['pointcloud'], filename)


    @staticmethod
    def numpy_from_pointcloud2_msg(msg):
        """

        :param msg: sensor_msgs/PointCloud2
        :type msg:
        :return:
        :rtype:
        """

        pc = ros_numpy.numpify(msg)
        num_points = msg.width * msg.height

        points = np.zeros((num_points, 3))
        points[:, 0] = pc['x'].flatten()
        points[:, 1] = pc['y'].flatten()
        points[:, 2] = pc['z'].flatten()

        return points









