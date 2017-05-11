import os
import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import director.applogic as app
import director.objectmodel as om
from director import visualization as vis
from director import transformUtils
from director import uipanel
from director import pointpicker
from director import applogic
from director import objectmodel as om
from director import vieweventfilter
from director import visualization as vis
from director import vtkAll as vtk
from director import segmentation as seg
from director import roboturdf as rob
from director import vtkNumpy as vtkn
from director.fieldcontainer import FieldContainer
from director.debugVis import DebugData
import numpy as np
from director import ioUtils
import json
from json import JSONEncoder

class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if hasattr(obj,'reprJSON'):
            return obj.reprJSON()
        else:
            return json.JSONEncoder.default(self, obj)

class Object():
    def __init__(self, name, pose):

        self.name = name
        self.pose = pose

    def reprJSON(self):
        return self.__dict__


class MyEventFilter(vieweventfilter.ViewEventFilter):

    def __init__(self, view, panel):
        vieweventfilter.ViewEventFilter.__init__(self, view)
        self.panel = panel

    def onMouseWheel(self, event):
        if self.panel.isModifyingBoundingBox:
            print event

    def onMouseMove(self, event):
        displayPoint = self.getMousePositionInView(event)
        self.panel.onMouseMove(displayPoint)

    def onLeftClick(self, event):
        displayPoint = self.getMousePositionInView(event)

        if event.modifiers() == QtCore.Qt.ShiftModifier:
            self.panel.onShiftMouseClick(displayPoint)

class GroundTruthAnnotation(uipanel.UiPanel):

    def __init__(self):
        #Loading
        filename = os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'apps/chris/ddGroundTruthAnnotationPanel.ui')
        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(filename)
        assert uifile.open(uifile.ReadOnly)
        self.widget = loader.load(uifile)
        uifile.close()

        #UI elements
        self.ui = WidgetDict(self.widget.children())
        self.view = app.getCurrentRenderView()
        self.ui.enabledCheck.connect('toggled(bool)', self.onEnabledCheckBox)
        self.ui.clearButton.connect('clicked()', self.onClear)
        self.ui.saveScene.connect('clicked()', self.saveCurrentScene)
        self.ui.align.connect('clicked()',self.vtkICP)

        self.eventFilter = MyEventFilter(view, self)
        self.annotation = vis.PolyDataItem('annotation', self.makeSphere((0,0,0)), view)
        self.annotation.setProperty('Color', [0,1,0])
        self.annotation.actor.SetPickable(False)
        self.annotation.actor.SetUserTransform(vtk.vtkTransform())
        self.setEnabled(False)

        #State
        self.pickPoints = []
        self.objects = []
        self.isModifyingBoundingBox = False
        #init folders
        self.getRootFolder()
        self.getModelObjectsFolder()
        self.getTransformedObjectsFolder()


    #UI functionality

    def onEnabledCheckBox(self):
        self.setEnabled(self.isEnabled())

    def isEnabled(self):
        return bool(self.ui.enabledCheck.checked)

    def setEnabled(self, enabled):
        self.ui.enabledCheck.checked = enabled

        self.annotation.setProperty('Visible', False)

        folder = self.getRootFolder(create=False)
        if folder:
            for obj in folder.children():
                obj.actor.SetPickable(not enabled)

        
        if self.isEnabled():
            self.eventFilter.installEventFilter()
        else:
            self.eventFilter.removeEventFilter()

        self.ui.panelContents.setEnabled(enabled)
    
    def saveCurrentScene(self):
        location = str(self.ui.saveLocation.text)
        with open(location+".json", 'w') as outfile:
            json.dump(self.objects, outfile, cls = MyEncoder)

        obj = om.findObjectByName("data files").children()[0]
        pd = obj.polyData
        ioUtils.writePolyData(pd, location+".vtk")
        print 'saved:', location+".vtk"

    def onClear(self):
        # self.ui.textEdit.clear()
        om.removeFromObjectModel(self.getRootFolder())
        om.removeFromObjectModel(self.getTransformedObjectsFolder())
        om.removeFromObjectModel(self.getModelObjectsFolder())

        self.pickPoints = []
        self.objects = []

    def pickIsValid(self):
        return self.ui.objName.text != 'none'

    def getRootFolder(self, create=True):
        name = 'scene objects'
        if create:
            return om.getOrCreateContainer(name)
        else:
            return om.findObjectByName(name)

    def getTransformedObjectsFolder(self, create=True):
        name = 'aligned objects'
        if create:
            return om.getOrCreateContainer(name)
        else:
            return om.findObjectByName(name)

    def getModelObjectsFolder(self, create=True):
        name = 'model objects'
        if create:
            return om.getOrCreateContainer(name)
        else:
            return om.findObjectByName(name)

    def makeSphere(self, position, radius=0.0075):
        d = DebugData()
        d.addSphere(position, radius=radius)
        return d.getPolyData()

    def snapshotGeometry(self):
        if not self.pickIsValid():
            return

        p = np.array([float(x) for x in self.ui.pickPt.text.split(', ')])
        self.pickPoints.append(p)

        obj = om.findObjectByName("data files").children()[0]
        pd = obj.polyData
        obj = seg.cropToSphere(pd,p.tolist(),.15)

        folder = self.getRootFolder()
        i = len(folder.children())
        obj = vis.showPolyData(obj, 'object %d' % i, color=[1,0,0], parent=folder)
        obj.actor.SetPickable(False)

    # def snapshotText(self):
    #     if not self.pickIsValid():
    #         return

    #     if len(self.pickPoints) > 1:
    #         dist = np.linalg.norm(np.array(self.pickPoints[-1].pose.position) - np.array(self.pickPoints[-2].pose.position))
    #     else:
    #         dist = 0.0

    #     s = 'pick_point ' + self.ui.pickPt.text + '\n'
    #     s += 'pick_normal ' + self.ui.pickNormal.text + '\n'
    #     s += 'dist_to_previous_point ' + '%f' % dist + '\n'
    #     s += '\n'

    #     self.ui.textEdit.append(s.replace('\n','<br/>'))

    def onShiftMouseClick(self, displayPoint):
        self.updatePick(displayPoint)
        self.snapshotGeometry()
        # self.snapshotText()
        self.annotation.setProperty('Visible', False)

    def onMouseMove(self, displayPoint):
        self.updatePick(displayPoint)

    def updatePick(self, displayPoint):

        pickType = str(self.ui.pickTypeCombo.currentText)
        if 'render' in pickType:
            pickType = 'render'
        elif 'vertex' in pickType:
            pickType = 'points'
        elif 'surface' in pickType:
            pickType = 'cells'
        else:
            raise Exception('unknown pick type')


        tolerance = self.ui.toleranceSpinBox.value
        pickPointFields = vis.pickPoint(
            displayPoint,
            self.view,
            pickType=pickType,
            tolerance=tolerance)

        worldPoint = pickPointFields.pickedPoint
        prop = pickPointFields.pickedProp
        dataset = pickPointFields.pickedDataset
        normal = pickPointFields.pickedNormal


        if not prop:
            worldPoint = np.zeros(3)
            normal = np.zeros(3)

        obj = vis.getObjectByProp(prop)

        self.ui.displayPt.text = '%d, %d' % tuple(displayPoint)
        self.ui.worldPt.text = '%.5f, %.5f, %.5f' % tuple(worldPoint)
        self.ui.pickPt.text = '%.5f, %.5f, %.5f' % tuple(worldPoint)
        self.ui.pickNormal.text = '%.5f, %.5f, %.5f' % tuple(normal)

        self.annotation.setProperty('Visible', prop is not None)
        t = vtk.vtkTransform()
        t.Translate(worldPoint)
        self.annotation.actor.SetUserTransform(t)
        self.annotation._renderAllViews()

        if obj:
            self.ui.objName.text = obj.getProperty('Name')
        else:
            self.ui.objName.text = 'none'

        if dataset:
            self.ui.numPts.text = dataset.GetNumberOfPoints()
            self.ui.numCells.text = dataset.GetNumberOfCells()
        else:
            self.ui.numPts.text = '0'
            self.ui.numCells.text = '0'

#model import/export

    def vtkMatrixToNumpy(self, matrix):
        output = np.ones((4,4))
        for i in range(4):
            for j in range(4):
                output[i, j] = matrix.GetElement(i, j)
        return output

    def getPolyDataByName(self,name):
            obj = om.findObjectByName(name).polyData

    def addURDFModel(self,path):
            model = rob.loadRobotModelFromFile(path)
            if model:
                model = rob.RobotModelItem(model)
                om.addToObjectModel(model,parentObj = self.getModelObjectsFolder())
                model.addToView(app.getCurrentRenderView())
            else:
                print "bad urdf model"
            #model = r.getObjectTree().getObjects()[1].polyData
            #vtkn.getNumpyFromVtk(model)
            # scene = om.findObjectByName(name)
            # if not scene is None :
            #     print "translating"
            #     t = vtk.vtkTransform()
            #     t.Translate(seg.computeCentroid(scene.polyData))
            #     scene.SetUserTransform(t)
    def addVTPModel(self,path):
        name = str(os.path.basename(path)).split(".")[0]
        polyData = ioUtils.readPolyData(path)
        vis.showPolyData(polyData, name ,color=[0,1,0], parent = self.getModelObjectsFolder())

    def packageAlignmentResult(self,object_name,transform):
        object = Object(object_name, self.vtkMatrixToNumpy(transform).tolist())
        self.objects.append(object)

    #alignment algorithms
    def vtkICP(self):
        object_name = str(self.ui.alignObject.text)
        if object_name == "" or not om.findObjectByName(object_name):
            print "object: " + object_name + " not found"
            return
        model = vtk.vtkPolyData()
        om.findObjectByName(object_name+".urdf").model.getModelMesh(model)
        scene = om.findObjectByName(object_name).polyData
        
        icp = vtk.vtkIterativeClosestPointTransform()
        icp.SetMaximumNumberOfIterations(100)

        #need to shift centroid to center of abject after clicking on it
        icp.StartByMatchingCentroidsOn()
        icp.SetSource(model)
        icp.SetTarget(scene)
        icp.GetLandmarkTransform().SetModeToRigidBody()
        icp.Modified()
        icp.Update()

        t = vtk.vtkTransformPolyDataFilter()
        t.SetInput(model)

        t.SetTransform(icp)
        t.Update()

        transformedObject = t.GetOutput()
        print transformedObject

        vis.showPolyData(transformedObject, object_name + "_transform" ,color=[0,1,0], parent = self.getTransformedObjectsFolder())
        self.packageAlignmentResult(object_name, icp.GetMatrix())

    def go_ICP(self):
        pass

    def pcl_ICP(self):
        pass


#add to UI
def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)




if __name__ == '__main__':
    print "starting ground truth panel"
    myWidget = GroundTruthAnnotation()
    myWidget.addURDFModel("/home/drc/spartan/apps/chris/cube.urdf")
    myWidget.widget.show()
