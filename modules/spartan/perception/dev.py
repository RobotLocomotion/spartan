#system
import numpy as np
import os

#director
from director import ioUtils
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


# spartan
import spartan.utils.utils as spartanUtils


#labelfusion
import labelfusion.cameraposes

def getFilenames(folderName, logName):
    """
    Parse some standard filenames into a dict given the logFolder
    :param logFolder:
    :return:
    """
    d = dict()
    d['logname'] = os.path.join(folderName, logName)
    d['cameraposes'] = os.path.join(folderName, "posegraph.posegraph")
    d['reconstruction'] = os.path.join(folderName, "reconstructed_pointcloud.vtp")

    return d



class ElasticFusionReconstruction(object):

	def __init__(self, folderName=None, logName="cup_on_table"):
		if folderName is None:
			sourceDir = spartanUtils.getSpartanSourceDir()
			folderName = os.path.join(sourceDir, 'logs', 'test')

		self.folderName = folderName
		self.logName = logName
		self.filenames = getFilenames(self.folderName, self.logName)
		self.loadReconstructedPointCloud(self.filenames['reconstruction'])

		self.cameraToWorldTransform = None
		self.setupSubscribers()


	def loadReconstructedPointCloud(self, filename=None):
		if filename is None:
			sourceDir = spartanUtils.getSpartanSourceDir()
			filename = os.path.join(sourceDir, 'logs', 'test', 'reconstructed_pointcloud.vtp')

		polyData = ioUtils.readPolyData(filename)
		self.pointcloud = vis.showPolyData(polyData, 'reconstructed pointcloud', colorByName='RGB')
		vis.addChildFrame(self.pointcloud)

	def loadCameraPoses(self, filename=None):
		if filename is None:
			filename = self.filenames['cameraposes']

		self.cameraPoses = labelfusion.cameraposes.CameraPoses(posegraphFile=filename)

	def setupSubscribers(self):
		lcmUtils.addSubscriber('OPENNI_FRAME_LEFT_TO_LOCAL', lcmbotcore.rigid_transform_t, self.onCameraToWorldTransform)

	def onCameraToWorldTransform(self,msg):
		# only do this once
		if self.cameraToWorldTransform is not None:
			return

		self.cameraToWorldTransform = lcmframe.frameFromRigidTransformMessage(msg)
		self.pointcloud.getChildFrame().copyFrame(self.cameraToWorldTransform)


