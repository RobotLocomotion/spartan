#system
import numpy as np

#director
from director import lcmUtils
from director import lcmframe
from director import transformUtils
from director import robotsystem
from director import segmentation
from director import cameraview
from director import pydrakeik
from director import packagepath
from director import roboturdf
from director import robotlinkselector
from director import fieldcontainer
from director import framevisualization
from director import drcargs
from director import visualization as vis
import director.objectmodel as om

import bot_core as lcmbotcore


# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.director_utils as spartanDirectorUtils




class CameraTransform(object):

    def __init__(self, robotSystem, referenceLinkName='palm', cameraToLinkTransform=None, channelName='OPENNI_FRAME_LEFT_TO_LOCAL', rgbCameraToLinkTransform=None):

        assert cameraToLinkTransform is not None

        self.robotSystem = robotSystem
        self.robotStateModel = self.robotSystem.robotStateModel
        self.referenceLinkName = referenceLinkName
        self.cameraToLinkTransform = cameraToLinkTransform
        self.channelName = channelName
        self.rgbCameraToLinkTransform = rgbCameraToLinkTransform
        self.setupSubscribers()

    def setupSubscribers(self):
        lcmUtils.addSubscriber("EST_ROBOT_STATE", lcmbotcore.robot_state_t, self.onEstRobotState)

    """
    Publishes the camera transform each time EST_ROBOT_STATE is received
    """
    def onEstRobotState(self, msg):

        cameraToWorld = self.getCameraToWorld()

        # make the message and publish it out
        cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorld)
        lcmUtils.publish(self.channelName, cameraToWorldMsg)

    def getCameraToWorld(self):
        linkFrame = self.robotStateModel.getLinkFrame(self.referenceLinkName) # this is a vtkTransform object
        cameraToWorld = transformUtils.concatenateTransforms([self.cameraToLinkTransform, linkFrame])
        return cameraToWorld

    def makeCameraFrameTeleop(self):
        self.cameraToWorld = self.getCameraToWorld()
        vis.updateFrame(self.cameraToWorld, 'camera frame teleop')

    def publishTeleopCameraFrame(self):
        cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(self.cameraToWorld)
        lcmUtils.publish(self.channelName, cameraToWorldMsg)

    def showCameraFrame(self):
        p = om.getOrCreateContainer('Camera Transform')
        cameraToWorld = self.getCameraToWorld()
        vis.updateFrame(cameraToWorld, 'depth camera frame', scale=0.15, parent=p)

    def showRGBCameraFrame(self):
        p = om.getOrCreateContainer('Camera Transform')
        linkFrame = self.robotStateModel.getLinkFrame(self.referenceLinkName) # this is a vtkTransform object
        cameraToWorld = transformUtils.concatenateTransforms([self.rgbCameraToLinkTransform, linkFrame])
        vis.updateFrame(cameraToWorld, 'rgb camera frame', scale=0.15, parent=p)

    def test(self):
        self.showCameraFrame()
        opticalFrame = self.getCameraToWorld()
        bodyFrame = CameraTransform.transformOpticalFrameToBodyFrame(opticalFrame)

        p = om.getOrCreateContainer('Camera Transform')
        vis.updateFrame(bodyFrame, 'depth camera body frame', scale=0.15, parent=p)

    @staticmethod
    def transformOpticalFrameToBodyFrame(opticalFrame):
        rpy = [-90,0,-90]
        opticalToBody = transformUtils.frameFromPositionAndRPY([0,0,0], rpy)
        bodyFrame = transformUtils.concatenateTransforms([opticalToBody.GetLinearInverse(), opticalFrame])
        return bodyFrame

    @staticmethod
    def fromConfigFilename(robotSystem, configFilename):
        config = spartanUtils.getDictFromYamlFilename(configFilename)

        transformDict = config['depth']['extrinsics']['transform_to_reference_link']
        cameraToLinkTransform = spartanDirectorUtils.transformFromPose(transformDict)


        rgbTransformDict = config['rgb']['extrinsics']['transform_to_reference_link']
        rgbCameraToLinkTransform = spartanDirectorUtils.transformFromPose(rgbTransformDict)

        return CameraTransform(robotSystem, referenceLinkName=config['depth']['extrinsics']['reference_link_name'], cameraToLinkTransform=cameraToLinkTransform, channelName=config['channel_name'], rgbCameraToLinkTransform=rgbCameraToLinkTransform)