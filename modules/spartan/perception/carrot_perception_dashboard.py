from functools import partial
import matplotlib.pyplot as plt
import numpy as np
import os
import struct
import trimesh
import sys
import yaml

from skimage.color import rgb2gray
from skimage.segmentation import felzenszwalb, slic, quickshift, watershed
from skimage.segmentation import mark_boundaries
from skimage.filters import sobel

from PyQt5 import (
    QtCore, QtGui, QtOpenGL, QtWidgets
    )
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QDoubleSpinBox,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QSpinBox,
    )
from PyQt5.QtGui import QIcon, QPixmap

import rospy
import sensor_msgs.msg
import ros_numpy
import tf

#import icp
#import cpd
import spartan.utils.mesh_creation as mesh_creation
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as ros_utils
import spartan.utils.transformations as transformations
from spartan.utils.QtImageViewer import QtImageViewer


def convertImagePointsToPointCloud(pt, camera_matrix):
    pt = np.array(pt).reshape(3, -1)
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    fx_inv = 1./camera_matrix[0, 0]
    fy_inv = 1./camera_matrix[1, 1]
    return pt[2]* np.array([fx_inv*(pt[0, :] - cx),
                            fy_inv*(pt[1, :] - cy),
                            np.ones(pt.shape[1])])

def convertDepthImageToPointCloud(depth_im, camera_matrix):
    X, Y = np.meshgrid(range(640), range(480))
    XYZ = np.vstack([X.ravel(), Y.ravel(), depth_im.ravel()])
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    fx_inv = 1./camera_matrix[0, 0]
    fy_inv = 1./camera_matrix[1, 1]
    pc = XYZ[2, :]*np.vstack([fx_inv*(XYZ[0, :] - cx),
                              fy_inv*(XYZ[1, :] - cy),
                              np.ones(XYZ.shape[1])])
    return pc

class App(QWidget):

    def __init__(self):
        super(App, self).__init__()
        rospy.init_node('carrot_perception_dashboard')
        self.vis = None
        #self.vis = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")["fitting_util"]
        #self.vis.delete()
        self.title = 'Carrot Perception Dashboard'
        self.left = 10
        self.top = 10
        self.width = 800
        self.height = 600
        self.initData()
        self.initUI()

    def initData(self):
        self.base_frame = "base"
        self.rgb_camera_frame = "camera_carmine_1_rgb_optical_frame"
        self.tf_listener = tf.TransformListener()

        self.camera_base_channel = "/camera_carmine_1"
        self.registered_cloud_subscriber = ros_utils.SimpleSubscriber(
            self.camera_base_channel + "/depth_registered/points",
            sensor_msgs.msg.PointCloud2)

        all_subscribers = [
            self.registered_cloud_subscriber
        ]

        for subscriber in all_subscribers:
            subscriber.start(queue_size=1)
            rospy.loginfo("Waiting on topic %s..." % (subscriber.topic))
            subscriber.waitForNextMessage()
        rospy.loginfo("All channels are alive.")

        
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
    
        mainLayout = QVBoxLayout()

        instruction_label = QLabel(
            "RIGHT CLICK AND DRAG to zoom in.\n"
            "DOUBLE RIGHT CLICK to zoom out.\n"
            "LEFT CLICK AND DRAG to pan.\n")
        instruction_label.setStyleSheet(
            " font-size: 20px; "
            " qproperty-alignment: AlignJustify; "
            " font-family: Courier New;")
        mainLayout.addWidget(instruction_label)

        # Create widget
        self.viewer = QtImageViewer()
        self.viewer.aspectRatioMode = QtCore.Qt.KeepAspectRatio
        self.viewer.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.viewer.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.viewer.canZoom = True
        # Allow panning with left mouse button.
        self.viewer.canPan = True

        self.viewer.leftMouseButtonPressed.connect(self.handleLeftClickOnImage)
        mainLayout.addWidget(self.viewer)

        dataControlLayout = QHBoxLayout()
        self.button_grab = QPushButton('Grab Current RGBD Frame')
        self.button_grab.clicked.connect(partial(
            self.handleGrabButton))
        dataControlLayout.addWidget(self.button_grab)
        mainLayout.addLayout(dataControlLayout)
 
        self.setLayout(mainLayout)
        self.show()

    def updateData(self, new_pc2):
        pc = ros_numpy.numpify(new_pc2)
        h = pc.shape[0]
        w = pc.shape[1]
        points=np.zeros([h, w, 3])
        points[:,:,0] = pc['x']
        points[:,:,1] = pc['y']
        points[:,:,2] = pc['z']

        good_mask = (np.isfinite(points[:, :, 2]) + 1.0)/2.0

        split_rgb_pc = ros_numpy.point_cloud2.split_rgb_field(pc)
        self.current_rgb_image = np.zeros([h, w, 3], dtype=np.int8)
        self.current_rgb_image[:,:,0] = split_rgb_pc['r']*good_mask
        self.current_rgb_image[:,:,1] = split_rgb_pc['g']*good_mask
        self.current_rgb_image[:,:,2] = split_rgb_pc['b']*good_mask
        qimage = QtGui.QImage(
            self.current_rgb_image,
            self.current_rgb_image.shape[1],
            self.current_rgb_image.shape[0],
            self.current_rgb_image.shape[1] * 3,
            QtGui.QImage.Format_RGB888)
        
        self.viewer_pixmap = QtGui.QPixmap(qimage)
        self.viewer.setImage(self.viewer_pixmap)

        self.current_depth_image = points[:, :, 2]
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.base_frame, self.rgb_camera_frame, rospy.Time(0))
            self.current_camera_pose = transformations.euler_matrix(rot[0], rot[1], rot[2])
            self.current_camera_pose[:3, 3] = trans[:]
        except:
            self.current_camera_pose = np.eye(4)
            rospy.logwarn("Couldn't get TF from %s to %s" % (self.base_frame, self.rgb_camera_frame))

        # Make entire depth point cloud for this perspective...
        #pc = convertDepthImageToPointCloud(self.current_depth_image, self.camera_matrix)
        #self.current_camera_pose = spartanUtils.homogenous_transform_from_dict(
        #    self.pose_data[self.curr_image_index]["camera_to_world"])
        #pc = spartanUtils.apply_homogenous_transform_to_points(
        #    self.current_camera_pose, pc)
        #self.current_colors = self.current_rgb_image.reshape(480*640, 3, order='A').astype(np.float64) / 255.

    def handleGrabButton(self):
        rospy.loginfo("Waiting for next pointcloud...")
        new_pc2 = self.registered_cloud_subscriber.waitForNextMessage()
        rospy.loginfo("Processing newest point cloud.")
        self.updateData(new_pc2)

    def resetSegmentLabeling(self):
        self.selected_points = np.zeros([3, 0])
        self.selected_colors = np.zeros([3, 0])
        self.selected_camera_poses = []
        self.selected_depth_images = []

    def handleLeftClickOnImage(self, x, y):
        print "Clicked ", x, y
        x = int(x)
        y = int(y)
        if x >= 640 or x < 0 or y >= 480 or y < 0:
            print "Clicked out of bounds, ignoring."
            return
        # Project x, y into the depth cloud
        print "Clicked depth: ", self.current_depth_image[y, x]

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())
