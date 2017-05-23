from director import objectmodel as om
from director import ioUtils
from director import visualization as vis
from director import transformUtils
from director import filterUtils
from director import segmentation
from director import pointpicker
from director import lcmframe
from director import lcmUtils
from director import applogic
from director.debugVis import DebugData
from director import viewbehaviors
from director import vtkAll as vtk
from director import vtkNumpy as vnp
from director.shallowCopy import shallowCopy
import numpy as np

from director.tasks.imagebasedaffordancefit import ImageBasedAffordanceFit

import PythonQt
from PythonQt import QtGui, QtCore


distanceToMeshThreshold = 0.2

class ImageFitter(ImageBasedAffordanceFit):

    def __init__(self, parent):
        ImageBasedAffordanceFit.__init__(self, imageView=parent.cameraView, numberOfPoints=3)
        self.parent = parent
        self.pointCloudObjectName = 'openni point cloud'

    def getPointCloud(self):
        obj = om.findObjectByName(self.pointCloudObjectName)
        return obj.polyData if obj else vtk.vtkPolyData()

    def fit(self, polyData, points):
        self.parent.onImagePick(points)


def makeDebugPoints(points, radius=0.01):
    d = DebugData()
    for p in points:
        d.addSphere(p, radius=radius)
    return shallowCopy(d.getPolyData())


def computeLandmarkTransform(sourcePoints, targetPoints):
    '''
    Returns a vtkTransform for the transform sourceToTarget
    that can be used to transform the source points to the target.
    '''
    sourcePoints = vnp.getVtkPointsFromNumpy(np.array(sourcePoints))
    targetPoints = vnp.getVtkPointsFromNumpy(np.array(targetPoints))

    f = vtk.vtkLandmarkTransform()
    f.SetSourceLandmarks(sourcePoints)
    f.SetTargetLandmarks(targetPoints)
    f.SetModeToRigidBody()
    f.Update()

    mat = f.GetMatrix()
    t = vtk.vtkTransform()
    t.PostMultiply()
    t.SetMatrix(mat)
    return t


def computePointToSurfaceDistance(pointsPolyData, meshPolyData):

    cl = vtk.vtkCellLocator()
    cl.SetDataSet(meshPolyData)
    cl.BuildLocator()

    points = vnp.getNumpyFromVtk(pointsPolyData, 'Points')
    dists = np.zeros(len(points))


    closestPoint = np.zeros(3)
    closestPointDist = vtk.mutable(0.0)
    cellId = vtk.mutable(0)
    subId = vtk.mutable(0)

    for i in xrange(len(points)):
        cl.FindClosestPoint(points[i], closestPoint, cellId, subId, closestPointDist)
        dists[i] = closestPointDist

    return np.sqrt(dists)


class TestFitCamera(object):

    def __init__(self, robotSystem, cameraView):

        self.meshPoints = None
        self.imagePoints = None
        self.cameraView = cameraView

        self.robotMesh = vtk.vtkPolyData()
        robotSystem.robotStateModel.model.getModelMesh(self.robotMesh)
        self.robotBaseFrame = robotSystem.robotStateModel.getLinkFrame('base')

        self.view = PythonQt.dd.ddQVTKWidgetView()
        vis.showPolyData(self.robotMesh, 'robot mesh', view=self.view)

        self.imageFitter = ImageFitter(self)

        vis.showPolyData(self.imageFitter.getPointCloud(), 'pointcloud', view=self.view, colorByName='rgb_colors', visible=False)

        self.picker = pointpicker.PointPicker(self.view)
        self.picker.pickType = 'cells'
        self.picker.numberOfPoints = 3
        self.picker.annotationName = 'mesh annotation'
        self.picker.annotationFunc = self.onPickPoints
        self.picker.start()

        self.widget = QtGui.QWidget()
        layout = QtGui.QHBoxLayout(self.widget)
        layout.addWidget(self.cameraView.view)
        layout.addWidget(self.view)
        self.widget.resize(800, 400)
        self.widget.setWindowTitle('Camera Alignment Tool')
        self.widget.show()

        self.viewBehaviors = viewbehaviors.ViewBehaviors(self.view)
        applogic.resetCamera(viewDirection=[0,1,0], view=self.view)
        applogic.setCameraTerrainModeEnabled(self.view, True)

    def onImagePick(self, points):
        self.imagePoints = np.array(points)
        vis.showPolyData(makeDebugPoints(self.imagePoints), 'image pick points', color=[1,0,0], view=self.view)
        self.align()

    def onPickPoints(self, *points):
        self.meshPoints = np.array(points)
        vis.showPolyData(makeDebugPoints(self.meshPoints), 'mesh pick points', color=[0,1,0], view=self.view)
        self.align()

    def align(self):

        if self.meshPoints is None or self.imagePoints is None:
            return

        t1 = computeLandmarkTransform(self.imagePoints, self.meshPoints)
        polyData = filterUtils.transformPolyData(self.imageFitter.getPointCloud(), t1)

        vis.showPolyData(polyData, 'transformed pointcloud', view=self.view, colorByName='rgb_colors', visible=False)
        vis.showPolyData(filterUtils.transformPolyData(makeDebugPoints(self.imagePoints), t1), 'transformed image pick points', color=[0,0,1], view=self.view)

        boxBounds = [[-0.5,0.50], [-0.3,0.3], [0.15,1.5]] #xmin,xmax,  ymin,ymax,  zmin,zmax

        polyData = segmentation.cropToBounds(polyData, self.robotBaseFrame, [[-0.5,0.50],[-0.3,0.3],[0.15,1.5]])

        #arrayName = 'distance_to_mesh'
        #dists = computePointToSurfaceDistance(polyData, self.robotMesh)
        #vnp.addNumpyToVtk(polyData, dists, arrayName)
        #polyData = filterUtils.thresholdPoints(polyData, arrayName, [0.0, distanceToMeshThreshold])


        #polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
        #polyData = segmentation.applyEuclideanClustering(polyData, clusterTolerance=0.04)
        #polyData = segmentation.thresholdPoints(polyData, 'cluster_labels', [1,1])

        vis.showPolyData(polyData, 'filtered points for icp', color=[0,1,0], view=self.view, visible=True)

        t2 = segmentation.applyICP(polyData, self.robotMesh)

        vis.showPolyData(filterUtils.transformPolyData(polyData, t2), 'filtered points after icp', color=[0,1,0], view=self.view, visible=False)


        cameraToWorld = transformUtils.concatenateTransforms([t1, t2])
        polyData = filterUtils.transformPolyData(self.imageFitter.getPointCloud(), cameraToWorld)
        vis.showPolyData(polyData, 'aligned pointcloud', colorByName='rgb_colors', view=self.view, visible=True)

        cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorld)
        lcmUtils.publish('OPENNI_FRAME_LEFT_TO_LOCAL', cameraToWorldMsg)


def main(robotSystem, cameraView):

    global w
    w = TestFitCamera(robotSystem, cameraView)
