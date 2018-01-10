from director import robotsystem
from director.consoleapp import ConsoleApp
from director import visualization as vis
from director.debugVis import DebugData
from director import objectmodel as om
from director import vtkNumpy
import director.vtkAll as vtk
from director import ioUtils
from director import drcargs

import numpy as np
import os

import PythonQt
from PythonQt import QtGui, QtCore

try:
    from vtk.util import numpy_support
except ImportError:
    from paraview import numpy_support


class LinkWidget(object):

    def __init__(self, view, robotModel, externalForce):
        self.view = view
        self.robotModel = robotModel
        self.linkName = None
        self.linkDict = {}
        self.pickedPoint = None
        self.normal = None
        self.externalForce = externalForce
        self.cellCaptureMode = False

        self.contactPointName = None

    def start(self):
        self.installEventFilter()
        self.externalForce.startPublishing()

    def stop(self):
        self.removeEventFilter()

    def installEventFilter(self):

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.view.vtkWidget().installEventFilter(self.eventFilter)

        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseMove)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonPress)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onEvent)

    def removeEventFilter(self):
        self.view.vtkWidget().removeEventFilter(self.eventFilter)

    def onEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseMove:
            self.onMouseMove(vis.mapMousePosition(obj, event), event.modifiers())
        elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
            self.onLeftMousePress(vis.mapMousePosition(obj, event), event.modifiers())

    # need to return the cellID of the picke thing
    def getSelection(self, displayPoint):

        pickData = vis.pickPoint(displayPoint, self.view, pickType='cells', tolerance=0.0)

        pickedPoint = pickData.pickedPoint
        pickedDataset = pickData.pickedDataset
        normal = pickData.pickedNormal
        pickedCellId = pickData.pickedCellId

        if not pickedDataset:
            return None

        linkName = self.robotModel.model.getLinkNameForMesh(pickedDataset)

        for name, data in self.linkDict.iteritems():
            if pickedDataset == data['polyData']:
                linkName = name

        if not linkName:
            return None

        return pickedPoint, linkName, normal, pickedCellId


    def onMouseMove(self, displayPoint, modifiers=None):

        om.removeFromObjectModel(om.findObjectByName('link selection'))
        self.linkName = None
        self.pickedPoint = None


        selection = self.getSelection(displayPoint)
        if selection is None:
            return

        pickedPoint, linkName, normal, pickedCellId = selection

        d = DebugData()
        d.addSphere(pickedPoint, radius=0.01)
        d.addLine(pickedPoint, np.array(pickedPoint) + 0.1 * np.array(normal), radius=0.005)
        obj = vis.updatePolyData(d.getPolyData(), 'link selection', color=[0,1,0])
        obj.actor.SetPickable(False)

        self.linkName = linkName
        self.pickedPoint = pickedPoint
        self.normal = normal
        self.pickedCellId = pickedCellId

        modifiers = QtGui.QApplication.keyboardModifiers()

        if modifiers == QtCore.Qt.ControlModifier and self.cellCaptureMode:
            print "provisionally capturing cell"
            self.provisionallyCaptureCell(linkName, pickedCellId)

        if modifiers == QtCore.Qt.ShiftModifier and self.cellCaptureMode:
            print "removing cell"
            self.removeCell(linkName, pickedCellId)


    def onLeftMousePress(self, displayPoint, modifiers=None):
        selection = self.getSelection(displayPoint)
        if selection is None:
            return
        pickedPoint, linkName, normal, pickedCellId = selection


        # add this as an external force with magnitude zero
        if self.linkName is not None:
            print ""
            print "added external force"
            print linkName, pickedPoint, normal
            print ""
            forceDirection = -np.array(normal)
            forceLocation = np.array(pickedPoint)
            forceMagnitude = self.externalForce.options['externalForce']['initialForceMagnitude']
            self.externalForce.addForce(linkName, forceDirection=forceDirection, forceLocation=forceLocation, forceMagnitude=forceMagnitude, inWorldFrame=True, name=self.contactPointName)
            self.contactPointName = None

    def setContactPointName(self, name):
        self.contactPointName = name


    def startCellCaptureMode(self):
        self.cellCaptureMode = True
        # make sure we start the event filter
        self.installEventFilter()

        linkNames = self.robotModel.model.getLinkNames()
        self.linkDict = {}
        for linkName in linkNames:
            polyData = vtk.vtkPolyData()
            self.robotModel.model.getLinkModelMesh(linkName, polyData)


            numCells = int(polyData.GetNumberOfCells())
            if numCells == 0:
                continue
            cellData = polyData.GetCellData()
            colorCodeArray = vtk.vtkIdTypeArray()
            colorCodeArray.SetNumberOfTuples(numCells)
            colorCodeArray.FillComponent(0,0)
            colorCodeArray.SetName("colorCodeArray")
            cellData.AddArray(colorCodeArray)
            cellData.SetActiveScalars("colorCodeArray")
            visObj = vis.showPolyData(polyData, linkName)
            mapper = visObj.mapper
            mapper.SetScalarModeToUseCellData()
            mapper.UseLookupTableScalarRangeOn()
            mapper.SetLookupTable(visObj._getDefaultColorMap(colorCodeArray, scalarRange=[0.0,2.0]))
            mapper.ScalarVisibilityOn()
            data = {'polyData': polyData, 'colorCodeArray': colorCodeArray, 'numCells': numCells,
                    'visObj': visObj, 'mapper': mapper, 'cellData': cellData}
            self.linkDict[linkName] = data


    def stopCellCaptureMode(self):
        self.cellCaptureMode = False
        # remove the event filter as well
        self.removeEventFilter()
        pass


    def provisionallyCaptureCell(self, linkName, cellId):

        currentVal = int(self.linkDict[linkName]['colorCodeArray'].GetValue(cellId))
        if currentVal == 1:
            return
        self.linkDict[linkName]['colorCodeArray'].SetValue(cellId, 2)
        self.linkDict[linkName]['colorCodeArray'].Modified()

    def captureAllCells(self):
        for linkName, data in self.linkDict.iteritems():
            colorCodeArray = data['colorCodeArray']

            colorCodeNumpy = numpy_support.vtk_to_numpy(data['colorCodeArray'])
            colorCodeNumpy.fill(1)
            data['colorCodeArray'].Modified()

    def provisionallyCaptureAllCellsOnLink(self, linkName):
        data = self.linkDict[linkName]
        colorCodeNumpy = numpy_support.vtk_to_numpy(data['colorCodeArray'])
        colorCodeNumpy.fill(2)
        data['colorCodeArray'].Modified()


    def captureProvisionalCells(self):
        for linkName, data in self.linkDict.iteritems():
            colorCodeNumpy = numpy_support.vtk_to_numpy(data['colorCodeArray'])
            colorCodeNumpy.clip(0,1, out=colorCodeNumpy)
            # colorCodeArrayNew = numpy_support.numpy_to_vtk(colorCodeNumpy)
            # data['colorCodeArray'].DeepCopy(colorCodeArrayNew)
            data['colorCodeArray'].Modified()

    def discardProvisionalCells(self):
        for linkName, data in self.linkDict.iteritems():
            colorCodeNumpy = numpy_support.vtk_to_numpy(data['colorCodeArray'])
            idx = np.where(colorCodeNumpy==2)[0]
            colorCodeNumpy[idx] = 0
            # colorCodeArrayNew = numpy_support.numpy_to_vtk(colorCodeNumpy)
            # data['colorCodeArray'].DeepCopy(colorCodeArrayNew)
            data['colorCodeArray'].Modified()

    """
    Switches a cell from captured to not
    """
    def removeCell(self, linkName, cellId):
        currentVal = int(self.linkDict[linkName]['colorCodeArray'].GetValue(cellId))
        if currentVal == 0: # return if it's already not captured
            return
        self.linkDict[linkName]['colorCodeArray'].SetValue(cellId, 0)
        self.linkDict[linkName]['colorCodeArray'].Modified()


    def manageKeyPressCapture(self):
        pass


    def saveCapturedCellsToFile(self, filename=None, overwrite=False):

        spartan_source_dir = os.getenv('SPARTAN_SOURCE_DIR')

        if filename is None:
            fullFilename = spartan_source_dir + self.externalForce.options['data']['contactCells']
        else:
            fullFilename = spartan_source_dir + \
                           "/drake/examples/ContactParticleFilter/config/" + filename

        dataDict = {}
        for linkName, data in self.linkDict.iteritems():
            cellCodeArray = numpy_support.vtk_to_numpy(data['colorCodeArray'])
            cellCodeArray = np.clip(cellCodeArray, 0, 1)
            dataDict[linkName] = cellCodeArray

        ioUtils.saveDataToFile(fullFilename, dataDict, overwrite=overwrite)

    def loadCapturedCellsFromFile(self, filename=None):

        spartan_source_dir = os.getenv('SPARTAN_SOURCE_DIR')

        if filename is None:
            fullFilename = spartan_source_dir + self.externalForce.options['data']['contactCells']
        else:
            fullFilename = spartan_source_dir + \
                           "/drake/examples/ContactParticleFilter/config/" + filename

        if len(self.linkDict) == 0:
            raise Exception("linkDict is empty, call startCellCaptureMode and then"
                            "try again")

        dataDict = ioUtils.readDataFromFile(fullFilename)

        for linkName, data in dataDict.iteritems():
            cellCodeArray = numpy_support.numpy_to_vtk(data)
            linkName = str(linkName)

            self.linkDict[linkName]['colorCodeArray'].DeepCopy(cellCodeArray)
            self.linkDict[linkName]['colorCodeArray'].Modified()


