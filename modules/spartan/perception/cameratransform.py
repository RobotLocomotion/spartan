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
import bot_core as lcmbotcore



pos = np.array([  1.38777878e-17,  -5.87267617e-02,   3.12766745e-02])
quat = np.array([  6.99802244e-01,   7.14336629e-01,   4.87254201e-17,
         5.41479628e-17])

cameraToPalm = transformUtils.transformFromPose(pos,quat)

class CameraTransform(object):

	def __init__(self, robotSystem, referenceLinkName='palm', cameraToLinkTransform=cameraToPalm, channelName='OPENNI_FRAME_LEFT_TO_LOCAL'):

		assert cameraToLinkTransform is not None

		self.robotSystem = robotSystem
		self.robotStateModel = self.robotSystem.robotStateModel
		self.referenceLinkName = referenceLinkName
		self.cameraToLinkTransform = cameraToLinkTransform
		self.channelName = channelName
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


