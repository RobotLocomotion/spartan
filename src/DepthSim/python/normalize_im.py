import os 
import numpy as np
from scipy import misc

path  = "/media/drc/DATA1/chris_labelfusion/logs_test/test_gan/train_real/"
#path  = "/media/drc/DATA1/chris_labelfusion/logs_test/test_gan_sim/train_log/"
for f in os.listdir(path):
	n = misc.imread(path+f)
	n = misc.imresize(n,(100,100))
	n=n/float(np.max(n))
	print n
	break
	misc.imsave(path+f, n)