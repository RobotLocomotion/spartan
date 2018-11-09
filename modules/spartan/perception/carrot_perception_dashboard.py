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
    QCheckBox,
    QDoubleSpinBox,
    QLabel,
    QGridLayout,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QSpinBox,
    QListWidget,
    QListWidgetItem,
    QStyle
    )
from PyQt5.QtGui import (
    QIcon, QPixmap, QFont
)

import rospy
import sensor_msgs.msg
import std_msgs.msg
import ros_numpy
import tf
import interactive_markers.interactive_marker_server as ros_im
import interactive_markers.menu_handler as ros_mh
from visualization_msgs.msg import (
    Marker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback
)

#import icp
#import cpd
import spartan.utils.mesh_creation as mesh_creation
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as ros_utils
import spartan.utils.transformations as transformations
from spartan.utils.QtImageViewer import QtImageViewer
import carrot_msgs.msg

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

class LabeledDoubleSpinBox(QWidget):
    def __init__(self, value, dmin, dmax, singlestep, decimals, label):
        super(LabeledDoubleSpinBox, self).__init__()
        layout = QHBoxLayout()
        self.label = QLabel(label)
        layout.addWidget(self.label)
        self.spinbox = QDoubleSpinBox(self)
        self.spinbox.setRange(dmin, dmax)
        self.spinbox.setSingleStep(singlestep)
        self.spinbox.setDecimals(decimals)
        self.spinbox.setValue(value)
        layout.addWidget(self.spinbox)
        self.setLayout(layout)

class CarrotHypothesisWidget(QWidget):
    def __init__(self, name, height, radius, owner_hypothesis):
        super(CarrotHypothesisWidget, self).__init__()
        layout = QGridLayout()
        self.name_label = QLabel(name)
        font = QFont()
        font.setBold(True)
        self.name_label.setFont(font)
        layout.addWidget(self.name_label, 0, 0)

        self.edit_toggle = QCheckBox("Edit")
        self.edit_toggle.setTristate(False)
        self.edit_toggle.setChecked(True)
        self.edit_toggle.toggled.connect(owner_hypothesis._handleEditClicked)
        layout.addWidget(self.edit_toggle, 0, 1)

        self.remove_button = QPushButton()
        self.remove_button.setIcon(self.style().standardIcon(QStyle.SP_DialogCloseButton))
        self.remove_button.clicked.connect(owner_hypothesis._handleRemove)
        layout.addWidget(self.remove_button, 0, 2)

        self.height_spinbox = LabeledDoubleSpinBox(
            height, dmin=0.001, dmax=0.1, singlestep=0.001, decimals=3, label="height")
        layout.addWidget(self.height_spinbox, 1, 0, 1, 3)
        self.height_spinbox.spinbox.valueChanged.connect(owner_hypothesis._handleParameterChangeCb)

        self.radius_spinbox = LabeledDoubleSpinBox(
            radius, dmin=0.001, dmax=0.1, singlestep=0.001, decimals=3, label="Radius")
        layout.addWidget(self.radius_spinbox, 2, 0, 1, 3)
        self.radius_spinbox.spinbox.valueChanged.connect(owner_hypothesis._handleParameterChangeCb)

        self.setLayout(layout)

    def getRadius(self):
        return self.radius_spinbox.spinbox.value()

    def getHeight(self):
        return self.height_spinbox.spinbox.value()

    def getEditState(self):
        return self.edit_toggle.isChecked()


class CarrotHypothesis():
    def __init__(self, tf, height, radius,
                 name, color, im_server, listwidget,
                 remove_callback):
        self.tf = tf.copy()
        self.height = height
        self.radius = radius
        self.color = color
        self.name = name

        # Users provide feedback to refine the mesh
        # position + configuration through RViz
        # InteractiveMarkers

        self.im_marker = ros_im.InteractiveMarker()
        self.im_marker.header.frame_id = "base"
        self.im_marker.name = name
        self.im_marker.description = "Hypothesized Carrot"
        self.im_marker.scale = 0.15
        self.im_marker.pose = ros_utils.ROSPoseMsgFromPose(spartanUtils.dict_from_homogenous_transform(tf))

        # Visualize current carrot mesh
        self.mesh_control = None
        self._regenerateMesh()

        # Add some control widgets
        self._add6DofControls()

        self.im_server = im_server
        self.im_server.insert(self.im_marker, self._processImMarkerFeedbackCb)
        self.im_server.applyChanges()

        # Make control interface + add to list widget
        self.control_widget = CarrotHypothesisWidget(
            name=name, radius=radius, height=height, owner_hypothesis=self)
        self.control_widget_listwidgetitem = QListWidgetItem(listwidget)
        self.control_widget_listwidgetitem.setSizeHint(self.control_widget.sizeHint())

        self.listwidget = listwidget
        self.listwidget.addItem(self.control_widget_listwidgetitem)
        self.listwidget.setItemWidget(self.control_widget_listwidgetitem, self.control_widget)
        self.remove_callback = remove_callback

    def populateCarrotConfigurationMessage(self, msg):
        msg.radius = self.radius
        msg.height = self.height
        msg.pose = self.im_marker.pose
        return msg

    def _handleRemove(self):
        self.im_server.erase(self.name)
        self.im_server.applyChanges()
        self.listwidget.takeItem(self.listwidget.row(self.control_widget_listwidgetitem))
        self.remove_callback(self)

    def _handleEditClicked(self):
        in_edit_mode = self.control_widget.getEditState()
        if in_edit_mode:
            # Add the 6DOF controls
            self._add6DofControls()
        else:
            for control in self.axis_controls:
                self.im_marker.controls.remove(control)
            self.axis_controls = []
        self._forceIntMarkerUpdate()

    def _forceIntMarkerUpdate(self):
        # Force update of the mesh
        update = ros_im.UpdateContext()
        update.update_type = ros_im.UpdateContext.FULL_UPDATE
        update.int_marker = self.im_marker
        self.im_server.pending_updates[self.name] = update 
        self.im_server.applyChanges()

    def _handleParameterChangeCb(self):
        self.height = self.control_widget.getHeight()
        self.radius = self.control_widget.getRadius()
        self._regenerateMesh()
        self._forceIntMarkerUpdate()

    def _processImMarkerFeedbackCb(self, feedback):
        if feedback.marker_name == self.im_marker.name:
            if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
                self.im_marker.pose = feedback.pose

    def _add6DofControls(self):
        self.axis_controls = []
        # Insert 6DOF control
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.im_marker.controls.append(control)
        self.axis_controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.im_marker.controls.append(control)
        self.axis_controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.im_marker.controls.append(control)
        self.axis_controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.im_marker.controls.append(control)
        self.axis_controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.im_marker.controls.append(control)
        self.axis_controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.im_marker.controls.append(control)
        self.axis_controls.append(control)

    def _regenerateMesh(self):
        self.mesh = mesh_creation.create_cut_cylinder(
              radius=self.radius,
              height=self.height,
              cutting_planes=[([0., 0., 0.], [1., 0., 0.])],
              sections=10)

        mesh_marker = Marker()
        mesh_marker.type = Marker.TRIANGLE_LIST
        mesh_marker.color.r = self.color[0]
        mesh_marker.color.g = self.color[1]
        mesh_marker.color.b = self.color[2]
        if len(self.color) > 3:
            mesh_marker.color.a = self.color[3]
        else:
            mesh_marker.color.a = 1.
        tris = self.mesh.faces.ravel()
        verts = self.mesh.vertices[tris, :]
        mesh_marker.points = ros_utils.arrayToPointMsgs(verts.T)
        mesh_marker.colors = [
            std_msgs.msg.ColorRGBA(
                self.color[0], self.color[1], self.color[2], self.color[3])
            ] * tris.shape[0]

        if self.mesh_control is not None:
            self.im_marker.controls.remove(self.mesh_control)
        self.mesh_control = InteractiveMarkerControl()
        self.mesh_control.always_visible = True
        self.mesh_control.markers.append(mesh_marker)
        self.im_marker.controls.append(self.mesh_control)
            

class App(QWidget):

    def __init__(self):
        super(App, self).__init__()
        rospy.init_node('carrot_perception_dashboard')
        self.im_server = ros_im.InteractiveMarkerServer("carrot_perception_dashboard")
        self.pub = rospy.Publisher('/carrot_configs', carrot_msgs.msg.CarrotConfigurationBundle, queue_size=1)

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
        self.base_frame = "/base"
        self.rgb_camera_frame = "/camera_carmine_1_rgb_optical_frame"
        self.tf_listener = tf.TransformListener()
        self.hypothesis_k = 0
        self.current_depth_image = None

        self.camera_base_channel = "/camera_carmine_1"
        self.registered_cloud_subscriber = ros_utils.SimpleSubscriber(
            self.camera_base_channel + "/depth_registered/points",
            sensor_msgs.msg.PointCloud2)
        self.rgb_camera_info_subscriber = ros_utils.SimpleSubscriber(
            self.camera_base_channel + "/depth_registered/sw_registered/camera_info",
            sensor_msgs.msg.CameraInfo)

        all_subscribers = [
            self.registered_cloud_subscriber,
            self.rgb_camera_info_subscriber
        ]

        for subscriber in all_subscribers:
            subscriber.start(queue_size=1)
            rospy.loginfo("Waiting on topic %s..." % (subscriber.topic))
            subscriber.waitForNextMessage()
        rospy.loginfo("All channels are alive.")

        camera_matrix_msg = self.rgb_camera_info_subscriber.waitForNextMessage()
        self.camera_matrix = np.array(camera_matrix_msg.K).reshape(3, 3)

        self.hypotheses = []
        
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
    
        mainLayout = QGridLayout()

        instruction_label = QLabel(
            "RIGHT CLICK AND DRAG to zoom in.\n"
            "DOUBLE RIGHT CLICK to zoom out.\n"
            "LEFT CLICK AND DRAG to pan.\n")
        instruction_label.setStyleSheet(
            " font-size: 20px; "
            " qproperty-alignment: AlignJustify; "
            " font-family: Courier New;")
        mainLayout.addWidget(instruction_label, 0, 0, 1, 2)

        self.viewer = QtImageViewer()
        self.viewer.aspectRatioMode = QtCore.Qt.KeepAspectRatio
        self.viewer.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.viewer.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.viewer.canZoom = True
        # Allow panning with left mouse button.
        self.viewer.canPan = True

        self.viewer.leftMouseButtonPressed.connect(self.handleLeftClickOnImage)
        mainLayout.addWidget(self.viewer, 1, 0, 1, 1)
        self.listwidget = QListWidget()
        mainLayout.addWidget(self.listwidget, 1, 1, 1, 1)

        self.button_grab = QPushButton('Grab Current RGBD Frame')
        self.button_grab.clicked.connect(partial(
            self.handleGrabButton))
        mainLayout.addWidget(self.button_grab, 2, 0, 1, 1)
 

        export_layout = QVBoxLayout()
        self.button_publish = QPushButton('Publish Pose Bundle')
        self.button_publish.clicked.connect(partial(
            self.handlePublishButton))
        export_layout.addWidget(self.button_publish)

        self.button_quick_export = QPushButton('Export Quick')
        self.button_quick_export.clicked.connect(partial(
            self.handleExportQuick))
        export_layout.addWidget(self.button_quick_export)

        self.button_export = QPushButton('Export As')
        self.button_export.clicked.connect(partial(
            self.handleExportAs))
        export_layout.addWidget(self.button_export)

        export_widget = QWidget()
        export_widget.setLayout(export_layout)
        mainLayout.addWidget(export_widget, 2, 1, 1, 1)

        self.setLayout(mainLayout)
        self.show()

    def handlePublishButton(self):
        if len(self.hypotheses) > 0:
            print "Publishing..."
            msg = carrot_msgs.msg.CarrotConfigurationBundle()
            msg.header = std_msgs.msg.Header()
            msg.header.frame_id = "base"
            msg.header.stamp = rospy.Time.now()
            msg.header.seq = 0

            for hyp in self.hypotheses:
                config_msg = carrot_msgs.msg.CarrotConfiguration()
                config_msg.header = msg.header
                config_msg = hyp.populateCarrotConfigurationMessage(config_msg)
                msg.configurations.append(config_msg)

            rospy.loginfo(msg)
            self.pub.publish(msg)
        else:
            rospy.logwarn("Not publishing, because there are no hypotheses.")


    def handleExportQuick(self):
        print "Export Quick"

    def handleExportAs(self):
        print "Export As"

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
            self.current_camera_pose = transformations.quaternion_matrix(np.array([rot[3], rot[0], rot[1], rot[2]]))
            self.current_camera_pose[:3, 3] = trans[:]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.current_camera_pose = np.eye(4)
            rospy.logwarn("Couldn't get TF from %s to %s" % (self.base_frame, self.rgb_camera_frame))

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

    def removeHypothesis(self, hyp):
        self.hypotheses.remove(hyp)

    def handleLeftClickOnImage(self, x, y):
        x = int(x)
        y = int(y)
        if x >= 640 or x < 0 or y >= 480 or y < 0:
            return
        if self.current_depth_image is None:
            return

        camera_frame_point = convertImagePointsToPointCloud(
            [x, y, self.current_depth_image[y, x]],
            self.camera_matrix)
        # And transform to world frame
        world_frame_point = spartanUtils.apply_homogenous_transform_to_points(
            self.current_camera_pose, camera_frame_point)        

        tf = np.eye(4)
        tf[:3, 3] = world_frame_point[:, 0]
        name = "carrot_%d" % self.hypothesis_k
        self.hypothesis_k += 1
        color = plt.cm.jet(np.random.random())
        height = 0.02
        radius = 0.02 
        new_hyp = CarrotHypothesis(tf, height, radius, name,
                                   color, self.im_server,
                                   self.listwidget,
                                   self.removeHypothesis)
        self.hypotheses.append(new_hyp)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())
