"""
This class consumes and lcmlog, extracts the images and saves them
to png
"""
import utils as CorlUtil
import os
import glob
import numpy as np
import scipy.misc

# director imports
import director.vtkAll as vtk
import director.vtkNumpy as vnp
from director import filterUtils
from director import lcmUtils
from director import cameraview
from director import mainwindowapp
from director import visualization as vis

# lcm imports
import bot_core as lcmbotcore



class OnlinePipeline(object):

    def __init__(self, imageManager, imageDir, cameraName):
        self.imageManager = imageManager
        self.imageDir = imageDir
        self.cameraName = cameraName

    def initSubscriber(self):
        lcmUtils.addSubscriber('OPENNI_FRAME', lcmbotcore.images_t(),
                               self.onImageMessage)

    def onImageMessage(self, msg):
        pass

    def processImage(self):

        # get the image and it's utime
        self.imageManager.updateImages()
        image = self.imageManager.getImage(self.cameraName)
        utime = self.imageManager.getUtime(self.cameraName)
        print "utime:", utime


def getImageFile(imageId, imageDir, suffix):
    filename = os.path.join(imageDir, '%010d' % imageId + '_%s' % suffix)
    print filename
    assert os.path.isfile(filename)
    return filename

def getUtimeFile(imageId, imageDir):
    return getImageFile(imageId, imageDir, suffix='utime.txt')

def getLabelFile(imageId, imageDir):
    return getImageFile(imageId, imageDir, suffix='labels.png')

def processImages(logFolder):

    corlPaths = CorlUtil.getFilenames(logFolder)

    lcmLogFilename = corlPaths['lcmlog']
    imageDir = corlPaths['images']

    channelName = 'OPENNI_FRAME'
    cameraName = 'OPENNI_FRAME_LEFT'

    imageManager = cameraview.ImageManager()
    imageManager.queue.addCameraStream(channelName, cameraName, lcmbotcore.images_t.LEFT)
    imageManager.addImage(cameraName)
    imageManager.queue.openLCMFile(lcmLogFilename)

    onlinePipeline = OnlinePipeline(imageManager, imageDir, cameraName)


    app = mainwindowapp.construct()


    for i in xrange(10):
        imageManager.queue.readNextImagesMessage()

        imageManager.updateImages()
        utime = imageManager.getUtime(cameraName)
        print utime

        polyData = vtk.vtkPolyData()

        decimation = 1
        removeSize = 0
        rangeThreshold = -1
        imageManager.queue.getPointCloudFromImages(channelName, polyData, decimation, removeSize, rangeThreshold);

        print polyData.GetNumberOfPoints()
        assert utime == int(open(getUtimeFile(i+1, imageDir), 'r').read())

        labelImageFile = getLabelFile(i+1, imageDir)

        img = scipy.misc.imread(labelImageFile)
        assert img.dtype == np.uint8
        assert img.ndim == 2
        img.shape = img.shape[0] * img.shape[1]
        assert img.shape[0] == polyData.GetNumberOfPoints()
        labelIds = np.unique(img)

        pointLabels = np.zeros(img.shape[0])

        for labelId in labelIds:
            pixelInds = img == labelId
            pointLabels[pixelInds] = labelId

        vnp.addNumpyToVtk(polyData, pointLabels, 'labels')

        for labelId in labelIds:
            cluster = filterUtils.thresholdPoints(polyData, 'labels', [labelId, labelId])
            vis.showPolyData(cluster, 'label %d' % labelId, parent='image %d' % i, colorByName='rgb_colors', visible=False)

    vis.showPolyData(polyData, 'pointcloud', colorByName='labels')

    app.app.start()

