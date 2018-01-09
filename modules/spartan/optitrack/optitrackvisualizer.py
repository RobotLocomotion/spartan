from director import affordanceitems
from director import applogic
from director import callbacks
from director import lcmUtils
from director import objectmodel as om
from director import visualization as vis
from director import vtkAll as vtk
from director import transformUtils
from director.debugVis import DebugData
from director.shallowCopy import shallowCopy
from director.thirdparty import transformations
from director import vtkAll as vtk

from optitrack import optitrack_data_descriptions_t
from optitrack import optitrack_frame_t

import numpy as np

# Test code below
from optitrack import optitrack_marker_set_t
from optitrack import optitrack_marker_t
from optitrack import optitrack_rigid_body_t
from optitrack import optitrack_rigid_body_description_t

test_message = optitrack_frame_t()

body = optitrack_rigid_body_t()
body.id = 100
body.xyz = [-0.1, 0.3, 0.6]
body.quat = [1., 0., 0., 0.]
marker_xyz = [[0.1, 0.1, 0.0], [-0.1, 0.1, 0.02], [-0.1, -0.1, -0.0]]
body.marker_xyz = [[xyz[0] + body.xyz[0], xyz[1] + body.xyz[1], xyz[2] + body.xyz[2]] for xyz in marker_xyz]
body.marker_ids = [101, 102, 103]
body.num_markers = len(body.marker_ids)
test_message.rigid_bodies.append(body)
test_message.num_rigid_bodies = len(test_message.rigid_bodies)

test_desc_message = optitrack_data_descriptions_t()
body_desc = optitrack_rigid_body_description_t()
body_desc.name = "Foo"
body_desc.id = 100
test_desc_message.rigid_bodies.append(body_desc)
test_desc_message.num_rigid_bodies = len(test_desc_message.rigid_bodies)


class OptitrackVisualizer(object):
    '''
    Usage:
      optitrackVis = OptitrackVisualizer('OPTITRACK_CHANNEL_NAME')
      # You can enable visualization of edges between optitrack markers,
      # but this visualization is slower.  To enable:
      optitrackVis.drawEdges = True
      # By default the lcm update rate is throttled to 10 hz.
      # To increase the update rate:
      optitrackVis.subscriber.setSpeedLimit(100)
      # Note, the constructor calls initSubscriber() automatically.
      # To remove the lcm subscriber:
      optitrackVis.removeSubscriber()
    '''

    defaultOptitrackToWorld = transformUtils.frameFromPositionAndRPY(
        [0,0,0],[90,0,90])

    def __init__(self, channel="OPTITRACK_FRAMES",
                 desc_channel="OPTITRACK_DATA_DESCRIPTIONS", name='Optitrack Visualier'):
        self.name = name
        self.channel = channel
        self.desc_channel = desc_channel
        self.subscriber = None
        self.desc_subscriber = None
        self.unitConversion = 0.001
        self.data_descriptions = None
        self.marker_sets = om.getOrCreateContainer(
            "Marker Sets", parentObj=self.getRootFolder())
        self.rigid_bodies = om.getOrCreateContainer(
            "Rigid Bodies", parentObj=self.getRootFolder())
        self.labeled_markers = om.getOrCreateContainer(
            "Labeled Markers", parentObj=self.getRootFolder())
        self.unlabeled_markers = om.getOrCreateContainer(
            "Unlabeled Markers", parentObj=self.getRootFolder())
        self.drawEdges = False
        self.markerGeometry = None
        self.optitrackToWorld = vtk.vtkTransform()
        self.optitrackToWorld.SetMatrix(
            self.defaultOptitrackToWorld.GetMatrix())
        self.callbacks = callbacks.CallbackRegistry([
            'RIGID_BODY_LIST_CHANGED',
            ])

    def connectRigidBodyListChanged(self, func):
        return self.callbacks.connect('RIGID_BODY_LIST_CHANGED', func)

    def initSubscriber(self):
        self.subscriber = lcmUtils.addSubscriber(
            self.channel, optitrack_frame_t, self.onMessage)
        self.subscriber.setSpeedLimit(10)
        self.desc_subscriber = lcmUtils.addSubscriber(
            self.desc_channel, optitrack_data_descriptions_t,
            self.onDescMessage)

    def removeSubscriber(self):
        if self.subscriber is not None:
            lcmUtils.removeSubscriber(self.subscriber)
            self.subscriber = None

        if self.desc_subscriber is not None:
            lcmUtils.removeSubscriber(self.desc_subscriber)
            self.desc_subscriber = None

    def isEnabled(self):
        return self.subscriber is not None

    def setEnabled(self, enabled):
        if enabled and not self.isEnabled():
            self.initSubscriber()
        elif not enabled and self.isEnabled():
            self.removeSubscriber()
            self.removeRootFolder()

    def enable(self):
        self.setEnabled(True)

    def disable(self):
        self.setEnabled(False)

    def getRootFolder(self):
        folder = om.getOrCreateContainer(self.channel)
        return folder

    def removeRootFolder(self):
        om.removeFromObjectModel(self.getRootFolder())

    def getMarkerGeometry(self):
        if self.markerGeometry is None:
            d = DebugData()
            d.addSphere(np.zeros(3), radius=0.007, resolution=8)
            self.markerGeometry = shallowCopy(d.getPolyData())

        return self.markerGeometry

    def createMarkerObjects(self, marker_ids, modelFolder, modelName, modelColor):

        geom = self.getMarkerGeometry()
        visible = modelFolder.getProperty('Visible')

        def makeMarker(i):
            obj = vis.showPolyData(
                shallowCopy(geom), modelName + ' marker %d' % i, color=modelColor, parent=modelFolder)
            obj.setProperty('Visible', visible)
            vis.addChildFrame(obj)
            return obj

        return [makeMarker(i) for i in marker_ids]

    def setRobotBaseTransform(self, transform):
        self.optitrackToWorld.Translate(transform.GetInverse().GetPosition())
        # Move down a little to account for the fact that the markers
        # aren't at the base of the robot.
        self.optitrackToWorld.Translate(0, 0.025, 0)
        # TODO(sam.creasey) handle rotation

    def _updateMarkerCollection(self, prefix, folder, marker_ids,
                                positions, base_transform=None):
        markers = folder.children()
        if len(markers) != len(positions):
            for obj in markers:
                om.removeFromObjectModel(obj)
            modelColor = vis.getRandomColor()
            markers = self.createMarkerObjects(
                marker_ids, folder, prefix, modelColor)
        if len(markers):
            modelColor = markers[0].getProperty('Color')

        for i, pos in enumerate(positions):
            marker_frame = transformUtils.transformFromPose(pos, (1, 0, 0, 0))
            marker_frame = transformUtils.concatenateTransforms(
                [marker_frame, self.optitrackToWorld])
            if base_transform is not None:
                marker_frame = transformUtils.concatenateTransforms(
                    [marker_frame, base_transform])
            markers[i].getChildFrame().copyFrame(marker_frame)

        # TODO(sam.creasey) we could try drawing edges here

    def _handleMarkerSets(self, marker_sets):
        # Get the list of existing marker sets so we can track any
        # which disappear.
        remaining_set_names = set(
            [x.getProperty('Name') for x in self.marker_sets.children()])

        for marker_set in marker_sets:
            set_name = 'Marker set ' + marker_set.name
            remaining_set_names.discard(set_name)
            set_folder = om.getOrCreateContainer(
                set_name, parentObj=self.marker_sets)
            marker_ids = range(marker_set.num_markers)
            self._updateMarkerCollection(
                marker_set.name + '.', set_folder, marker_ids, marker_set.xyz)

        for remaining_set in remaining_set_names:
            obj = om.findObjectByName(remaining_set, self.marker_sets)
            om.removeFromObjectModel(obj)

    def _handleRigidBodies(self, rigid_bodies):
        # Get the list of existing rigid bodies so we can track any
        # which disappear.
        remaining_body_names = set(
            [x.getProperty('Name') for x in self.rigid_bodies.children()])
        bodies_added = False
        bodies_removed = False

        for body in rigid_bodies:
            body_name = 'Body ' + str(body.id)
            for desc in self.data_descriptions.rigid_bodies:
                if desc.id == body.id:
                    body_name = desc.name
            if body_name in remaining_body_names:
                body_obj = om.findObjectByName(
                    body_name, parent=self.rigid_bodies)
            else:
                bodies_added = True
                # The use of a box here is arbitrary.
                body_obj = affordanceitems.BoxAffordanceItem(
                    body_name, applogic.getCurrentRenderView())
                om.addToObjectModel(body_obj, parentObj=self.rigid_bodies)
                vis.addChildFrame(body_obj).setProperty('Deletable', False)
                body_obj.setProperty('Surface Mode', 'Wireframe')
                body_obj.setProperty('Color', [1,0,0])
            remaining_body_names.discard(body_name)

            x,y,z,w = body.quat
            quat = (w,x,y,z)
            objToOptitrack = transformUtils.transformFromPose(body.xyz, quat)

            # Dimension our box based on a bounding across all of our
            # markers.
            all_xyz = body.marker_xyz + [body.xyz]
            all_xyz = [(xyz[0] - body.xyz[0], xyz[1] - body.xyz[1], xyz[2] - body.xyz[2])
                       for xyz in all_xyz]
            marker_transforms = [transformUtils.transformFromPose(xyz, (1, 0, 0, 0))
                                 for xyz in all_xyz]
            inv_transform = transformUtils.transformFromPose((0, 0, 0), (w, -x, -y, -z))
            marker_transforms = [transformUtils.concatenateTransforms([t, inv_transform])
                                 for t in marker_transforms]
            all_xyz = [t.GetPosition() for t in marker_transforms]
            (min_x, min_y, min_z) = (
                min(xyz[0] for xyz in all_xyz),
                min(xyz[1] for xyz in all_xyz),
                min(xyz[2] for xyz in all_xyz))
            (max_x, max_y, max_z) = (
                max(xyz[0] for xyz in all_xyz),
                max(xyz[1] for xyz in all_xyz),
                max(xyz[2] for xyz in all_xyz))
            dimensions = (
                max(0.01, max_x - min_x),
                max(0.01, max_y - min_y),
                max(0.01, max_z - min_z))
            #print "max, min", (max_x, max_y, max_z), (min_x, min_y, min_z)
            body_obj.setProperty('Dimensions', dimensions)
            objToWorld = transformUtils.concatenateTransforms([objToOptitrack, self.optitrackToWorld])
            body_obj.getChildFrame().copyFrame(objToWorld)

            folder = om.getOrCreateContainer('Models', parentObj=body_obj)
            self._updateMarkerCollection(
                body_name + '.', folder, body.marker_ids,
                body.marker_xyz)#, base_transform=transform)

        if len(remaining_body_names):
            bodies_removed = True

        for remaining_body in remaining_body_names:
            obj = om.findObjectByName(remaining_body, self.rigid_bodies)
            om.removeFromObjectModel(obj)

        if bodies_added or bodies_removed:
            self.callbacks.process(
                'RIGID_BODY_LIST_CHANGED',
                sorted([x.getProperty('Name')
                        for x in self.rigid_bodies.children()]))

    def _handleLabeledMarkers(self, labeled_markers):
        marker_ids = [x.id for x in labeled_markers]
        marker_positions = [x.xyz for x in labeled_markers]
        # We'll rename the items ourselves later.
        self._updateMarkerCollection(
            "dummy prefix", self.labeled_markers, marker_ids, marker_positions)
        for i, marker in enumerate(self.labeled_markers.children()):
            marker_name = 'Marker ' + str(marker_ids[i])
            if marker.getProperty('Name') != marker_name:
                marker.rename(marker_name)

    def _handleUnlabeledMarkers(self, positions):
        self._updateMarkerCollection(
            "Unlabeled", self.unlabeled_markers, range(len(positions)),
            positions)

    def onMessage(self, msg):
        self.lastMessage = msg
        if self.data_descriptions is None:
            return
        self._handleMarkerSets(msg.marker_sets)
        self._handleRigidBodies(msg.rigid_bodies)
        #self._handleLabeledMarkers(msg.labeled_markers)
        #self._handleUnlabeledMarkers(msg.other_markers)

    def onDescMessage(self, msg):
        self.data_descriptions = msg