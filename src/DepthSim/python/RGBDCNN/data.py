import numpy as np
import matplotlib.pyplot as plt
import os
from scipy import misc
from keras.preprocessing import image
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Dropout
from keras.layers import Flatten
from keras.constraints import maxnorm
from keras.optimizers import SGD
from keras.layers.convolutional import Convolution2D
from keras.layers.convolutional import MaxPooling2D
from keras.layers import Activation
from keras.layers import Reshape
from keras.layers.convolutional import Conv2D
from keras.layers import *
from keras import Model
from keras import optimizers
from keras.callbacks import ModelCheckpoint, LearningRateScheduler
import matplotlib.pyplot as plt
from keras.utils import np_utils
from keras import backend as K
import cv2
from sklearn.model_selection import train_test_split

from keras.preprocessing import image
from itertools import izip

def hot_vectorize(x):
    zero_mask = x==0
    non_zero_mask = x!=0
    x[zero_mask] = 1
    x[non_zero_mask] = 0
    return x

def combine_data_generators(depth,normal,rgb,mask):
    while True:
            d = depth.next()
            n = normal.next()
            r = rgb.next()
            m = mask.next()
            yield (np.concatenate((d,n,r),3),m)

def generate_data(depth_as_mask=True,img_height=480,img_width=640,batch_size=2,rgb="/media/drc/DATA/CNN/rgb",depth="/media/drc/DATA/CNN/depth",normal="/media/drc/DATA/CNN/normal",gtdepth="/media/drc/DATA/CNN/gtdepth"):      
	

	datagen = image.ImageDataGenerator(
	        #preprocessing_function=applications.xception.preprocess_input,
	        rescale =1/255.,
	        data_format="channels_last")

	labeldatagen = image.ImageDataGenerator(
	        #preprocessing_function=applications.xception.preprocess_input,
	        preprocessing_function = hot_vectorize,
	        data_format="channels_last") if depth_as_mask else datagen

	flow_from_directory_params_grey = {'target_size': (img_height,img_width),
	                                  'color_mode': 'grayscale',
	                                  'class_mode': None,
	                                  'batch_size': batch_size}

	flow_from_directory_params_color = {'target_size': (img_height,img_width),
	                                  'color_mode': 'rgb',
	                                  'class_mode': None,
	                                  'batch_size': batch_size}


	mask_generator = labeldatagen.flow_from_directory(
	        directory=depth,
	        shuffle = False,
	        **flow_from_directory_params_grey
	    )
	rgb_generator = datagen.flow_from_directory(
	        directory=rgb,
	        shuffle = False,
	        **flow_from_directory_params_color
	    )
	normal_generator = datagen.flow_from_directory(
	        directory=normal,
	        shuffle = False,
	        **flow_from_directory_params_color
	    )
	gt_generator = datagen.flow_from_directory(
	        directory=gtdepth,
	        shuffle =False,
	        **flow_from_directory_params_grey
	    )
	return combine_data_generators(gt_generator,normal_generator,rgb_generator,mask_generator)