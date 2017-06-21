import os
from computeIntersectionOverUnion import computeIOUfile
import numpy as np


class ComputeIoUHelper(object):

    def __init__(self, dir_full_path):
        self.N = 2
        self.dir_full_path = dir_full_path
        self.trialsIOU = {}
        self.crawlDir()

    def printFinal(self):
        for k in self.trialsIOU.keys():
            print k

    def crawlDir(self):
        for filename in sorted(os.listdir(self.dir_full_path)):
            print filename
            split_filename = filename.split("_",1)
            if len(split_filename) < 2:
                continue

            img_num    = split_filename[0]
            identifier = split_filename[1] 

            # look for labels
            if identifier.startswith("labels"):
                #print "found labels"
                label_num  = img_num 
                label_file = filename
                continue

            # look for predlabels for each trial
            if identifier.startswith("predlabels"):

                if identifier not in self.trialsIOU.keys():
                    if int(img_num) > 1:
                        print "error: i don't have a starting image for all trials!"
                        quit()
                    self.trialsIOU[identifier] = {}

                #print "found predlabels"
                if img_num != label_num:
                    print "error: not matching"
                    quit()
                
                frameIOU = computeIOUfile(label_file, filename)
                #print "frameIOU type", type(frameIOU)
                #print "identifier", identifier
                for k, v in frameIOU.items():
                #print "k ", k, v
                #print self.trialsIOU[identifier]
                    if k not in self.trialsIOU[identifier].keys():
                #print "added for ", k
                        self.trialsIOU[identifier][k] = []
                #print self.trialsIOU[identifier][k]

                #print type(self.trialsIOU[identifier][k])
                self.trialsIOU[identifier][k].append(v)


if __name__ == '__main__':

    dir_full_path = os.getcwd()
    print dir_full_path

    compute_iou_helper = ComputeIoUHelper(dir_full_path)
    compute_iou_helper.printFinal()

    print compute_iou_helper

	