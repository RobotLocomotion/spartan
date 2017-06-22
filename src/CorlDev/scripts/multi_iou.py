import os
from computeIntersectionOverUnion import computeIOUfile
import numpy as np
import sys
sys.path.insert(0, "/home/peteflo/spartan/src/CorlDev/modules/")
from corl import utils
import corl.utils


# call this script from the directory which has both the *label*.png and *predlabels*.png files

class ComputeIoUHelper(object):

    def __init__(self, dir_full_path):
        self.dir_full_path = dir_full_path
        self.trialsIOU = {}
        self.crawlDir()

    def printAndSaveSummary(self):
        target = open("summary.txt", 'w')
        for trial_name, data in sorted(self.trialsIOU.iteritems()):
            print trial_name
            target.write(trial_name)
            target.write("\n")
            for label, iou_all_frames_list in sorted(data.iteritems()):
                print "label,", label, "object,", corl.utils.getObjectName(label), "mean iou", np.average(iou_all_frames_list)
                target.write(str(label) + " " + str(np.average(iou_all_frames_list)) + " " + str(np.std(iou_all_frames_list)) + " " + corl.utils.getObjectName(label))
                target.write("\n")
        target.close()

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

                if img_num != label_num:
                    print "error: not matching"
                    quit()
                
                frameIOU = computeIOUfile(label_file, filename)

                for k, v in frameIOU.items():
                    if k not in self.trialsIOU[identifier].keys():
                        self.trialsIOU[identifier][k] = []
                    self.trialsIOU[identifier][k].append(v)


if __name__ == '__main__':

    dir_full_path = os.getcwd()
    print dir_full_path


    compute_iou_helper = ComputeIoUHelper(dir_full_path)
    compute_iou_helper.printAndSaveSummary()

    print compute_iou_helper

	