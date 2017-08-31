import cv
import cv2
import sys
import math
import signal
import thread

# numerics
import numpy as np

# the messaging stuff
import lcm
from bot_core import image_t, images_t

def update_lcm(name, lc):
    while (1):
        try:
            lc.handle()
        except Exception as e:
            print "Exception ", e, " in lcm update loop."

def create_lcm():
    return lcm.LCM()

def start_lcm(lc):
    thread.start_new_thread( update_lcm, ("LCM Updater", lc) )

class ImagesHandler():
    def __init__(self, channel, N=1, lc=None, parent=None, name=None):
        self.N = N
        self.lc = lc
        if name:
            self.setObjectName(name)

        self.images = [None]*self.N  
        #self.setMinimumSize(640, 480)
        #self.setMaximumSize(1920, 1080)
        image_sub = self.lc.subscribe(channel, self.handle_images_t)

        for i in range(self.N):
            cv2.namedWindow(str(i), cv2.WINDOW_NORMAL)

    def update(self):
        for i in range(self.N):
            if self.images[i] is not None:
                cv2.imshow(str(i), self.images[i])


    def handle_images_t(self, channel, data):
        msg = images_t.decode(data)
        for i in range(min(self.N, msg.n_images)):
            self.handle_image_t(msg.images[i], i, msg.image_types[i]==images_t.DEPTH_MM_ZIPPED)
            print "Image %d: %d x %d" % (i, msg.images[i].height, msg.images[i].width)

    def handle_image_t(self, msg, i, gzip=False):
        # conversion black magic...
        if msg.pixelformat == image_t.PIXEL_FORMAT_MJPEG:
            source = cv2.imdecode(np.asarray(bytearray(msg.data), dtype="uint8"), flags=cv2.CV_LOAD_IMAGE_COLOR)
            self.images[i] = source
        elif msg.pixelformat == image_t.PIXEL_FORMAT_INVALID:
            data = msg.data
            if gzip:
                import zlib
                data = zlib.decompress(data)
            source = np.reshape(np.frombuffer(data, dtype="uint16"), (msg.height, msg.width))
            source = source.astype(float) / float(np.max(source))
            self.images[i] = source

        else:
            print "Don't know how to handle this image type: ", msg.pixelformat

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    lc = create_lcm()

    window = ImagesHandler("OPENNI_FRAME", N=2, lc=lc)
    
    start_lcm(lc)

    while(1):
        cv2.waitKey(1)
        window.update()