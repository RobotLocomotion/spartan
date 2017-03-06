import director
import math
import textwrap
import time
import drake as lcmdrake
import bot_core as lcmbotcore
import director.vtkAll as vtk
from collections import namedtuple
from director import transformUtils
from director import visualization as vis
from director import objectmodel as om
from director import lcmUtils
import robotlocomotion as robotlocomotion_lcmtypes
import cpf_lcmtypes

from director.debugVis import DebugData

from director.timercallback import TimerCallback
from director import ioUtils

import copy
import os
import os.path
import csv
import functools
import numpy as np
import scipy.io
import yaml

from director import drcargs
from director import utime as utimeUtil
from pythondrakemodel import PythonDrakeModel
from twostepestimator import TwoStepEstimator
import contactfilterutils as cfUtils

Wrench_Time = namedtuple('wrenchTime', ['wrench','time'])
class ExternalForce(object):

    def __init__(self, robotSystem, configFilename="contact_particle_filter_config.yaml"):
        self.robotSystem = robotSystem
        self.robotStateModel = robotSystem.robotStateModel
        self.robotStateModel.connectModelChanged(self.onModelChanged)
        self.options = cfUtils.loadConfig(configFilename)

        self.loadDrakeModelFromFilename()
        self.initializeRobotPoseTranslator()
        self.initializeJointNamesList()

        # keys = linkNames, wrench = 6 x 1 Torque-Force vector, all in body frame
        self.externalForces = dict()
        self.publishChannel = 'EXTERNAL_FORCE_TORQUE'
        self.captureMode = False
        self.captureModeCounter = 0
        self.showContactRay = True
        self.showContactFilterEstimate = True
        self.addSubscribers()
        self.createPlunger()
        self.createMeshDataAndLocators()

        self.createTwoStepEstimator(configFilename)

        self.visObjectDrawTime = dict()
        self.timeout = 1.5

        # setup timercallback to publish, lets say at 5 hz
        self.timer = TimerCallback(targetFps=25)
        self.timer.callback = self.publish
        self.startPublishing()
        # self.startPublishing() # this now gets activated when you start linkSelection

        self.visObjectCleanupTimer = TimerCallback(targetFps = 1)
        self.visObjectCleanupTimer.callback = self.removeStaleVisObjects
        self.visObjectCleanupTimer.start()

        self.initializationTime = time.time()

        self.trueResidual = None
        self.estimatedResidual = None

    def addSubscribers(self):
        # lcmUtils.addSubscriber('CONTACT_FILTER_POINT_ESTIMATE', cpf_lcmtypes.contact_filter_estimate_t, self.onContactEstimate)
        lcmUtils.addSubscriber('RESIDUAL_OBSERVER_STATE', cpf_lcmtypes.residual_observer_state_t, self.onResidualObserverState)


        # lcmUtils.addSubscriber("CONTACT_FILTER_BODY_WRENCH_ESTIMATE", cpf_lcmtypes.contact_filter_body_wrench_estimate_t, self.onActiveLinkContactEstimate)
        # lcmUtils.addSubscriber("EXTERNAL_FORCE_TORQUE", lcmdrake.lcmt_external_force_torque(), self.onActiveLinkContactEstimate)

    def createMeshDataAndLocators(self):
        self.linkMeshData = dict()

        drakeModelLinkNames = self.robotStateModel.model.getLinkNames()

        for linkName in drakeModelLinkNames:
            linkName = str(linkName)
            data = dict()

            polyData = vtk.vtkPolyData()
            self.robotStateModel.model.getLinkModelMesh(linkName, polyData)
            transform = self.robotStateModel.getLinkFrame(linkName)

            data['linkName'] = linkName
            data['polyData'] = polyData
            data['transform'] = transformUtils.copyFrame(self.robotStateModel.getLinkFrame(linkName))
            if (polyData.GetNumberOfCells() == 0):
                print linkName + " mesh has no cells, not building a locator for it"
                continue
            data['locator'] = self.buildCellLocator(polyData)
            self.linkMeshData[linkName] = data

    def createTwoStepEstimator(self, configFilename):
        self.twoStepEstimator = TwoStepEstimator(self.robotStateModel, self.robotSystem.robotStateJointController,
                                                 self.linkMeshData, configFilename)

    # either get the EST_ROBOT_STATE utime or just use the wall clock
    def getUtime(self):
        msg = self.robotSystem.robotStateJointController.lastRobotStateMessage
        utime = 0

        if msg is not None:
            utime = msg.utime
        else:
            utime = utimeUtil.getUtime()/5.0 # slow down time by factor of 5

        return utime

    def loadDrakeModelFromFilename(self, filename=None):
        urdf_filename = self.options['robot']['urdf']
        floatingBaseTypeString = self.options['robot']['floatingBaseType']

        self.drakeModel = PythonDrakeModel(floatingBaseTypeString, urdf_filename)

    def initializeJointNamesList(self):
        jointNamesTuple = self.drakeModel.model.getJointNames()
        jointNames = []
        for idx, val in enumerate(jointNamesTuple):
            jointNames.append(str(val))

        self.jointNames = jointNames

    def initializeRobotPoseTranslator(self):
        self.robotPoseTranslator = cfUtils.RobotPoseTranslator(self.robotSystem.robotStateModel.model, self.drakeModel.model)

    # linkName is a string, wrench is an np.array
    def addForce(self, linkName, wrench=None, forceDirection=None, forceMagnitude=None, forceLocation=None, inWorldFrame=False):

        linkName = str(linkName) # getting a weird u in front otherwise
        d = dict()
        # need at least one of wrench, or forceDirection and forceMagnitude
        assert (wrench is not None) or ((forceDirection is not None) and (forceMagnitude is not None) and (forceLocation is not None))

        if self.captureMode:
            self.captureModeCounter += 1
            key = linkName + "_" + str(self.captureModeCounter)
        else:
            key = linkName


        # check to see if a force on this body already exists, if so then use that as the forceMagnitude
        if self.externalForces.has_key(key):
            forceMagnitude = self.externalForces[key]['forceMagnitude']


        visName = key + ' external force'
        om.removeFromObjectModel(om.findObjectByName(visName))


        if wrench is not None:
            if inWorldFrame:
                raise ValueError('do not support specifying wrench in world frame')
            d['wrench'] = wrench
            d['forceLocation'] = np.array([0,0,0])
            d['forceDirection'] = wrench[3:]/np.linalg.norm(wrench[3:])
            d['forceMagnitude'] = np.linalg.norm(wrench[3:])
            d['isWrench'] = True
            d['linkName'] = linkName
        else:
            if inWorldFrame:
                linkToWorld = self.robotStateModel.getLinkFrame(linkName)
                worldToLink = linkToWorld.GetLinearInverse()
                forceLocation = np.array(worldToLink.TransformPoint(forceLocation))
                forceDirection = np.array(worldToLink.TransformDoubleVector(forceDirection))

            # this should all be in link frame
            forceDirection = forceDirection/np.linalg.norm(forceDirection)
            d['forceDirection'] = forceDirection
            d['forceMagnitude'] = forceMagnitude
            d['forceLocation'] = forceLocation
            d['isWrench'] = False
            d['linkName'] = linkName





        d['time'] = time.time()
        self.externalForces[key] = d
        self.updateContactWrench(key)
        self.drawForces()



    def computeWrench(self, linkName, forceDirection, forceMagnitude, forceLocation):
        outputFrame = vtk.vtkTransform()
        wrenchFrame = vtk.vtkTransform()
        wrenchFrame.Translate(forceLocation)

        forceMomentTransform = transformUtils.forceMomentTransformation(wrenchFrame, outputFrame)

        wrench = np.zeros(6)
        wrench[3:] = forceMagnitude*forceDirection
        wrenchTransformed = np.dot(forceMomentTransform, wrench)

        return wrenchTransformed

    # WARNING: make sure you call doKinematics before you get here
    def computeSingleContactPointResidual(self, linkName, wrench):
        linkId = self.drakeModel.model.findLinkID(linkName)
        geometricJacobian = self.drakeModel.geometricJacobian(0,linkId,linkId,0,False)
        singleContactResidual = np.dot(geometricJacobian.transpose(), wrench)
        return singleContactResidual


    def removeForce(self, key, callFromFrameObj=False):
        if not self.externalForces.has_key(key):
            return

        visObjectName = key + ' external force'
        self.externalForces.pop(key, None)

        if not callFromFrameObj:
            om.removeFromObjectModel(om.findObjectByName(visObjectName))

    def removeAllForces(self):
        keyList = list(self.externalForces.keys())

        for key in keyList:
            self.removeForce(key)



    # remove forces from dict that haven't been refreshed in at least self.timeout seconds
    def removeStaleExternalForces(self):
        keysToRemove = []
        for key, value in self.externalForces.iteritems():
            elapsed = time.time() - value['time']

            if elapsed > self.timeout:
                keysToRemove.append(key)


        for key in keysToRemove:
            self.removeForce(key)


    def removeStaleVisObjects(self):
        keysToRemove = []
        for key, val in self.visObjectDrawTime.iteritems():
            elapsed = time.time() - val
            if elapsed > self.timeout:
                keysToRemove.append(key)


        for key in keysToRemove:
            om.removeFromObjectModel(om.findObjectByName(key))
            del self.visObjectDrawTime[key]


    # be careful here if director and this use different models
    # for example if we are FIXED base and director has ROLLPITCHYAW
    def getCurrentPose(self):
        q_director = self.robotSystem.robotStateJointController.q
        q =self.robotPoseTranslator.translateDirectorPoseToRobotPose(q_director)
        return q

    def publish(self):

        # if len(self.externalForces) == 0:
        #     return

        tol = 1e-3
        numExternalForces = 0
        msg = cpf_lcmtypes.external_force_torque_t()

        msgMultipleContactLocations = cpf_lcmtypes.multiple_contact_location_t()
        trueResidual = np.zeros((self.drakeModel.numJoints,))

        # make sure we call doKinematics before we do all the geometricJacobian stuff
        if self.options['debug']['publishTrueResidual']:
            q = self.getCurrentPose()
            self.drakeModel.model.setJointPositions(q)

            # alternatively can just do setJointPositions
            # self.drakeModel.model.setJointPositions(q)


        for key, val in self.externalForces.iteritems():
            # don't publish it if the force is very small
            if np.linalg.norm(val['wrench']) < tol:
                continue

            numExternalForces += 1

            msg.body_names.append(val['linkName'])
            msg.tx.append(val['wrench'][0])
            msg.ty.append(val['wrench'][1])
            msg.tz.append(val['wrench'][2])
            msg.fx.append(val['wrench'][3])
            msg.fy.append(val['wrench'][4])
            msg.fz.append(val['wrench'][5])

            linkName = val['linkName']
            linkFrame = self.robotStateModel.getLinkFrame(linkName)
            contactLocationInWorld = linkFrame.TransformPoint(val['forceLocation'])
            contactNormalInWorld = linkFrame.TransformVector(val['forceDirection'])

            force = val['forceMagnitude']*val['forceDirection']
            forceInWorld = linkFrame.TransformPoint(force)

            msgContactLocation = cpf_lcmtypes.single_contact_filter_estimate_t()
            msgContactLocation.body_name = linkName
            msgContactLocation.contact_position = val['forceLocation']
            msgContactLocation.contact_force = force
            msgContactLocation.contact_normal = val['forceDirection']
            msgContactLocation.contact_position_in_world = contactLocationInWorld
            msgContactLocation.contact_force_in_world = forceInWorld
            msgContactLocation.contact_normal_in_world = contactNormalInWorld

            msgMultipleContactLocations.contacts.append(msgContactLocation)

            # compute the true residual if we are asked to publish it
            if self.options['debug']['publishTrueResidual']:
                singleContactResidual = self.computeSingleContactPointResidual(val['linkName'], val['wrench'])
                trueResidual += singleContactResidual

        # this message goes to the simulator
        msg.num_external_forces = numExternalForces
        lcmUtils.publish(self.publishChannel, msg)

        # this message is for analysis
        msgMultipleContactLocations.num_contacts = numExternalForces
        lcmUtils.publish("EXTERNAL_CONTACT_LOCATION", msgMultipleContactLocations)

        # this message is for debugging
        if self.options['debug']['publishTrueResidual']:
            self.trueResidual = trueResidual
            residualMsg = cpf_lcmtypes.residual_observer_state_t()
            residualMsg.utime = self.getUtime()
            residualMsg.num_joints = len(self.jointNames)
            residualMsg.joint_name = self.jointNames
            residualMsg.residual = trueResidual

            # these are just placeholders so lcm doesn't complain
            # we are not actually populating them
            residualMsg.gravity = 0*trueResidual
            residualMsg.internal_torque = 0*trueResidual
            residualMsg.foot_contact_torque = 0*trueResidual

            lcmUtils.publish("RESIDUAL_ACTUAL", residualMsg)

        if self.options['twoStepEstimator']['computeEstimate']:
            twoStepEstimateData = self.computeTwoStepEstimate()

            # if twoStepEstimateData is None it means that some criterion
            # wasn't satisfied and we didn't actually perform the estimation
            if twoStepEstimateData is not None:
                self.publishTwoStepEstimateData(twoStepEstimateData, msgMultipleContactLocations)

                if self.options['twoStepEstimator']['visualize']:
                    self.visualizeTwoStepEstimate(twoStepEstimateData)

    def visualizeTwoStepEstimate(self, data):
        for linkName, singleContactData in data.iteritems():
            self.visualizeTwoStepEstimateSingleContact(singleContactData)

    def visualizeTwoStepEstimateSingleContact(self, data):

        # draw the contact ray if that option is set
        if self.options['twoStepEstimator']['showContactRay']:
            d = DebugData()
            d.addLine(data['contactRay']['rayOriginInWorld'],data['contactRay']['rayEndInWorld'],
                      radius=0.005)
            color = data['contactRay']['color']
            visName = data['contactRay']['visObjectName']
            obj = vis.updatePolyData(d.getPolyData(), visName, color=color)
            self.visObjectDrawTime[visName] = time.time()


        # draw the actual contact point if it exists
        if data['pt'] is not None:
            visName = data['linkName'] +  " active link estimated external force"
            self.drawForce(visName, data['linkName'], data['pt'], data['force'], color=[1,0,0])
            self.visObjectDrawTime[visName] = time.time()



    def publishTwoStepEstimateData(self, twoStepEstimateData, actualContactLocationsMsg):
        msg = cpf_lcmtypes.actual_and_estimated_contact_locations_t()
        msg.utime = self.getUtime()
        msg.actual_contact_location = actualContactLocationsMsg

        estMsg = cpf_lcmtypes.multiple_contact_location_t()
        estMsg.num_contacts = len(twoStepEstimateData)

        for linkName, data in twoStepEstimateData.iteritems():
            tmpMsg = cpf_lcmtypes.single_contact_filter_estimate_t()
            tmpMsg.body_name = linkName
            tmpMsg.contact_normal = data['force']

            if data['pt'] is None:
                tmpMsg.utime = -1 # signifies that no intersection was found
            else:
                tmpMsg.contact_position = data['contactLocation']
                tmpMsg.contact_position_in_world = data['contactLocationInWorld']

            tmpMsg.contact_force = data['force']
            tmpMsg.contact_force_in_world = data['forceInWorld']
            estMsg.contacts.append(tmpMsg)

        msg.estimated_contact_location = estMsg

        lcmUtils.publish(self.options['twoStepEstimator']['publishChannel'], msg)

    def startPublishing(self):
        self.captureMode = False
        self.captureModeCounter = 0
        self.removeAllForces()
        self.timer.start()

    def stopPublishing(self):
        print "stopping publishing"
        self.timer.stop()

    def startCaptureMode(self):
        self.stopPublishing()
        print "starting capture mode"
        self.removeAllForces()
        self.captureMode = True
        self.captureModeCounter = 0

    def onResidualObserverState(self, msg):
        msgJointNames = msg.joint_name
        msgData = msg.residual
        residual = self.drakeModel.extractDataFromMessage(msgJointNames, msgData)
        self.estimatedResidual = residual

    def onContactEstimate(self, msg):

        if not self.showContactFilterEstimate:
            return

        name = 'estimated external force'

        for i in xrange(0,msg.num_contact_points):
            subMsg = msg.single_contact_estimate[i]
            forceLocation = np.array(subMsg.contact_position)
            force = np.array(subMsg.contact_force)

            eps = 0.5
            name = 'estimated external force ' + str(i)
            if np.linalg.norm(force) < eps:
                # om.removeFromObjectModel(om.findObjectByName(name))
                return

            self.drawForce(name, subMsg.body_name, forceLocation, force, color=[0,0,1])
            self.visObjectDrawTime[name] = time.time()


    def drawForce(self, name, linkName, forceLocation, force, color, key=None):

        forceDirection = force/np.linalg.norm(force)
        # om.removeFromObjectModel(om.findObjectByName(name))

        linkToWorld = self.robotStateModel.getLinkFrame(linkName)
        forceLocationInWorld = np.array(linkToWorld.TransformPoint(forceLocation))
        forceDirectionInWorld = np.array(linkToWorld.TransformDoubleVector(forceDirection))

        # point = forceLocationInWorld - 0.1*forceDirectionInWorld

        # d = DebugData()
        # # d.addArrow(point, forceLocationInWorld, headRadius=0.025, tubeRadius=0.005, color=color)
        # d.addSphere(forceLocationInWorld, radius=0.01)
        # d.addLine(point, forceLocationInWorld, radius=0.005)

        transformForVis = transformUtils.getTransformFromOriginAndNormal(forceLocationInWorld, forceDirectionInWorld)

        obj = vis.updatePolyData(self.plungerPolyData, name, color=color)
        obj.actor.SetUserTransform(transformForVis)


        if key is not None and om.findObjectByName(name) is not None:
            obj.addProperty('magnitude', self.externalForces[key]['forceMagnitude'])
            obj.addProperty('linkName', linkName)
            obj.addProperty('key', key)
            obj.connectRemovedFromObjectModel(self.removeForceFromFrameObject)

        obj.properties.connectPropertyChanged(functools.partial(self.onPropertyChanged, obj))
        return obj


    # connect this with an on model changed
    def drawForces(self):
        if len(self.externalForces) == 0:
            return

        for key, val in self.externalForces.iteritems():
            linkName = val['linkName']
            name = key + ' external force'
            #Green is for a force, red is for a wrench
            color = [0,1,0]
            if val['isWrench']:
                color = [1,0,0]

            # linkToWorld = self.robotStateModel.getLinkFrame(linkName)

            # forceLocationInWorld = np.array(linkToWorld.TransformPoint(val['forceLocation']))
            # forceDirectionInWorld = np.array(linkToWorld.TransformDoubleVector(val['forceDirection']))

            # point = forceLocationInWorld - 0.1*forceDirectionInWorld



            # d = DebugData()
            # # d.addArrow(point, forceLocationInWorld, headRadius=0.025, tubeRadius=0.005, color=color)
            # d.addSphere(forceLocationInWorld, radius=0.01)
            # d.addLine(point, forceLocationInWorld, radius=0.005)


            obj = self.drawForce(name, linkName, val['forceLocation'], val['forceDirection'], color, key=key)


    def onModelChanged(self, model):
        self.drawForces()

    def onPropertyChanged(self, frameObj, propertySet, propertyName):
        if propertyName != 'magnitude':
            return
        key = frameObj.getProperty('key')
        linkName = frameObj.getProperty('linkName')
        magnitude = frameObj.getProperty('magnitude')
        if magnitude < 0:
            print "you must specify a positive magnitude"
            print "external forces can only PUSH, NOT PULL"
            return

        self.externalForces[key]['forceMagnitude'] = magnitude
        self.updateContactWrench(key)

    def updateContactWrench(self, key):
        if not self.externalForces.has_key(key):
            return

        val = self.externalForces[key]

        # if it was specified as a wrench, then don't overwrite it
        if val['isWrench']:
            return

        val['wrench'] = self.computeWrench(val['linkName'], val['forceDirection'],  val['forceMagnitude'], val['forceLocation'])


    def removeForceFromFrameObject(self, tree_, frameObj):
        key = frameObj.getProperty('key')
        self.removeForce(key, callFromFrameObj=True)



    def computeTwoStepEstimate(self):
        residual = None
        if self.options['debug']['useTrueResidual']:
            residual = self.trueResidual
        else:
            residual = self.estimatedResidual

        # this means we haven't gotten any data yet
        # so just return an empty dict which means no data
        if residual is None:
            return None

        if self.options['noise']['addNoise']:
            residualSize = np.size(residual)
            residual = residual + np.random.normal(scale=self.options['noise']['stddev'], size=residualSize)


        if self.options['twoStepEstimator']['provideLinkContactInfo']:
            # only do this if we are using fake residual
            linksWithContactForce = []
            for key, val in self.externalForces.iteritems():
                if val['forceMagnitude'] > 0.1:
                    linksWithContactForce.append(key)
        else:
            linksWithContactForce=None


        return self.twoStepEstimator.computeTwoStepEstimate(residual, linksWithContactForce)


    def printForces(self):
        for key in self.externalForces.keys():
            print key

    # deprecated
    def saveForceLocationsToFileOld(self, filename=None, verbose=False, overwrite=False):

        spartan_source_dir = os.getenv('SPARTAN_SOURCE_DIR')

        if filename is None:
            fullFilename = spartan_source_dir + self.options['data']['contactCells']
        else:
            fullFilename = spartan_source_dir + \
                           "/src/ContactParticleFilter/config/" + filename


        print "saving initial particle locations to ", filename

        if os.path.isfile(fullFilename) and not overwrite:
            print "FILE ALREADY EXISTS, set the overwrite flag to true to overwrite"
            return

        fileObject = open(fullFilePath, 'w')
        for key, val in self.externalForces.iteritems():
            line = str(val['linkName']) + ","

            for i in range(0,3):
                line += str(val['forceLocation'][i]) + ","

            for i in range(0,3):
                line += str(val['forceDirection'][i]) + ","

            line += "\n"

            if verbose:
                print line

            fileObject.write(line)

        fileObject.close()


    def saveForceLocationsToFile(self, filename=None, verbose=False, overwrite=False):

        spartan_source_dir = os.getenv('SPARTAN_SOURCE_DIR')

        if filename is None:
            fullFilename = spartan_source_dir + self.options['data']['contactCells']
        else:
            fullFilename = spartan_source_dir + \
                           "/src/ContactParticleFilter/config/" + filename


        print "saving initial particle locations to ", fullFilename

        if os.path.isfile(fullFilename) and not overwrite:
            print "FILE ALREADY EXISTS, set the overwrite flag to true to overwrite"
            return

        ioUtils.saveDataToFile(fullFilename, self.externalForces, overwrite=overwrite)

    def addForcesFromFile(self, filename=None):
        self.startCaptureMode()
        spartan_source_dir = os.getenv('SPARTAN_SOURCE_DIR')

        if filename is None:
            fullFilename = spartan_source_dir + self.options['data']['initialParticleLocations']
        else:
            fullFilename = spartan_source_dir + \
                           "/src/ContactParticleFilter/config/" + filename

        dataDict = ioUtils.readDataFromFile(fullFilename)
        for key, val in dataDict.iteritems():
            linkName = val['linkName']
            forceLocation = val['forceLocation']
            forceDirection = val['forceDirection']
            self.addForce(linkName, wrench=None, forceDirection=forceDirection, forceMagnitude=0.0, forceLocation=forceLocation, inWorldFrame=False)

    def createPlunger(self):
        forceLocationInWorld = np.array([0,0,0])
        forceDirectionInWorld = np.array([0,0,1])

        point = forceLocationInWorld - 0.1*forceDirectionInWorld
        color = [1,0,0]
        d = DebugData()
        # d.addArrow(point, forceLocationInWorld, headRadius=0.025, tubeRadius=0.005, color=color)
        d.addSphere(forceLocationInWorld, radius=0.01)
        d.addLine(point, forceLocationInWorld, radius=0.005)
        self.plungerPolyData = d.getPolyData()


    @staticmethod
    def buildCellLocator(polyData):
        print "building cell locator"
        loc = vtk.vtkCellLocator()
        loc.SetDataSet(polyData)
        loc.BuildLocator()
        return loc

    @staticmethod
    def raycast(locator, rayOrigin, rayEnd):
        tolerance = 0.0 # intersection tolerance
        pt = [0.0, 0.0, 0.0] # data coordinate where intersection occurs
        lineT = vtk.mutable(0.0) # parametric distance along line segment where intersection occurs
        pcoords = [0.0, 0.0, 0.0] # parametric location within cell (triangle) where intersection occurs
        subId = vtk.mutable(0) # sub id of cell intersection

        result = locator.IntersectWithLine(rayOrigin, rayEnd, tolerance, lineT, pt, pcoords, subId)

        return pt if result else None


    def visualizeMesh(self, linkName):
        if linkName not in self.linkMeshData:
            print "I can't find a mesh corresponding to " + linkName
            return

        vis.showPolyData(self.linkMeshData[linkName]['polyData'], linkName + ' mesh')

    # these are test methods
    def setupTest(self):
        w = np.array([1,2,3,4,5,6])
        self.addForce('pelvis', wrench=w)
        self.startPublishing()

    def test1(self):
        forceDirection = np.array([0,0,1])
        forceMagnitude = 100
        forceLocation = np.array([0,0,0])
        linkName = 'pelvis'

        self.addForce(linkName, forceDirection=forceDirection, forceMagnitude=forceMagnitude, forceLocation=forceLocation)
        self.drawForces()

    def test2(self):
        wrench = np.array([0,0,0,0,0,100])
        linkName = 'pelvis'
        self.addForce(linkName, wrench=wrench)


    def constructTestFrames(self):
        T = vtk.vtkTransform();
        S = vtk.vtkTransform()
        S.Translate([1,2,0])
        FM = transformUtils.forceMomentTransformation(S,T)
        print FM
        return T,S, FM




