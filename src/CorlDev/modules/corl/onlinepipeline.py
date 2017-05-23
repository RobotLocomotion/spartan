"""
This class consumes and lcmlog, extracts the images and saves them
to png
"""
import utils as CorlUtil
import os
import glob

# director imports
import director.vtkAll as vtk
from director import filterUtils
from director import lcmUtils
from director import cameraview

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


def getImageFile(imageId, suffix, imageDir):
    filename = os.path.join(imageDir, '%010d' % imageId + '_%s.txt' % suffix)
    assert os.path.isfile(filename)
    return filename

def getUtimeFile(imageId, imageDir):
    return getImageFile(imageId, imageDir, suffix='utime')

def getLabelFile(imageId, imageDir):
    return getImageFile(imageId, imageDir, suffix='labels')

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

    for i in xrange(10):
        imageManager.queue.readNextImagesMessage()

        imageManager.updateImages()
        utime = imageManager.getUtime(cameraName)

        assert utime == int(open(getUtimeFile(i+1, imageDir), 'r').read())



