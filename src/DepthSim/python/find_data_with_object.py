import os 
import numpy as np
from scipy import misc

path  = "/home/drc/spartan/Data_ALL/logs_test/"
paths = []
for f in os.listdir(path):
	if "2017" in f:
		if "registration_result.yaml" in os.listdir(path+f):
			with open(path+f+"/registration_result.yaml") as read:
				if "drill" in read.read():
					paths.append(f) 
print paths
for i in paths:
	#os.system("directorPython depth.py /home/drc/spartan/Data_ALL/logs_test/"+i+"/" 4000 'None' "/home/drc/spartan/Data_ALL/drill/")

#do all drill and real depth data for all images