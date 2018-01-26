import os 
import numpy as np
from scipy import misc

path  = "/media/drc/DATA/chris_labelfusion/logs_test/test_gan/train_real/"
path1  = "/media/drc/DATA/chris_labelfusion/logs_test/test_gan_sim/train_log/"

for f in os.listdir(path):
	if ".png" in f:
		n = misc.imread(path+f)
		n=n/4000.
		misc.imsave(path+f, n)

for f in os.listdir(path1):
	if ".png" in f:
		n = misc.imread(path1+f)
		n=n/4000.
		misc.imsave(path1+f, n)