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
import glob

from keras.preprocessing import image
from itertools import izip
import sys
sys.path.insert(0,'../')
from common import util

def hot_vectorize(x):
    zero_mask = x==0
    non_zero_mask = x!=0
    x[zero_mask] = 1
    x[non_zero_mask] = 0
    return x

def stack_frames(frames,img_height,img_width,channels):
    stack  = np.zeros((1,img_height,img_width,channels))
    index = 0
    for i in frames:
        num_chan = 1 if len(np.shape(i)) == 2 else np.shape(i)[2]
        stack[0,:,:,index:index+num_chan] = np.reshape(i,(img_height,img_width,num_chan))
        index += num_chan
    return stack

def grab_frame(files,i,path,func=None):
    img = misc.imread(path+files[i])
    if func:
        return func(img)
    return img

def normalize(x):
    return x/255.

def normalize_depth(x):
    return x/3500.

def combine_data_generators(depth,normal,rgb,mask):
    while True:
            d = depth.next()
            n = normal.next()
            r = rgb.next()
            m = mask.next()
            yield (np.concatenate((d,n,r),3),m)

def generate_data(depth_as_mask=True,img_height=480,img_width=640,batch_size=2,path="/media/drc/DATA/CNN/",dir_name =""):      
	
	rgb = path+"rgb/"+dir_name
	depth = path+"depth/"+ dir_name
	gtdepth = path+"gtdepth/" + dir_name
	normal = path+"normal/" + dir_name

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

def generate_data_custom(img_height=480,img_width=640,batch_size=2,path="/media/drc/DATA/chris_labelfusion/CORL2017/test_data/",dir_name = "test/",filter_files = None,func=None):

	rgb_path = path+"rgb/rgb/"
	depth_path = path+"depth/depth/"
	gtdepth_path = path+"gtdepth/gtdepth/" # change back to + dir_name
	normal_path = path+"normal/normal/"

	rgb = np.sort(os.listdir(rgb_path))
	normal = np.sort(os.listdir(normal_path))
	depth = np.sort(os.listdir(depth_path))
	gtdepth = np.sort(os.listdir(gtdepth_path))

	if filter_files:
		rgb = np.sort(filter(lambda x: filter_files in x, rgb))
		normal = np.sort(filter(lambda x: filter_files in x, normal))
		depth = np.sort(filter(lambda x: filter_files in x, depth))
		gtdepth = np.sort(filter(lambda x: filter_files in x, gtdepth))

	i = -1
	while True:
		i+=1%len(rgb)
		rgb_img = grab_frame(rgb,i,rgb_path,func)
		normal_img = grab_frame(normal,i,normal_path,func)
		gtdepth_img = grab_frame(gtdepth,i,gtdepth_path,func)
		depth_img = grab_frame(depth,i,depth_path,func)
		stack = stack_frames([gtdepth_img,normal_img,rgb_img,depth_img],img_height,img_width,8)
		yield stack

def generate_data_custom2(depth_as_mask=True,img_height=480,img_width=640,batch_size=4,path="/media/drc/DATA/CNN/",dir_name = "test/",filter_files = None,func=None):

    #rgb_path = path+"rgb/"+"rgb/"
    depth_path = path+"depth/"+ "depth/"
    gtdepth_path = path+"gtdepth/" + "gtdepth/"
    normal_path = path+"normal/" + "normal/"

    #rgb = np.sort(os.listdir(rgb_path))
    normal = np.sort(os.listdir(normal_path))
    depth = np.sort(os.listdir(depth_path))
    gtdepth = np.sort(os.listdir(gtdepth_path))

    if filter_files:
            #rgb = np.sort(filter(lambda x: filter_files in x, rgb))
            normal = np.sort(filter(lambda x: filter_files in x, normal))
            depth = np.sort(filter(lambda x: filter_files in x, depth))
            gtdepth = np.sort(filter(lambda x: filter_files in x, gtdepth))

    i = -1
    while True:
        stack1 = np.zeros((batch_size,img_height,img_width,4))
        stack2 = np.zeros((batch_size,img_height,img_width,1))

        for j in range(batch_size):
            i= (i+1)%len(depth)
            #rgb_img = grab_frame(rgb,i,rgb_path,func)
            normal_img = grab_frame(normal,i,normal_path,normalize)
            gtdepth_img = grab_frame(gtdepth,i,gtdepth_path,normalize_depth)
            depth_img = grab_frame(depth,i,depth_path,hot_vectorize)
            a = util.bounding_box(gtdepth_img,img_height)
            while not a:
            	print a
                i= (i+1)%len(depth)
                #rgb_img = grab_frame(rgb,i,rgb_path,func)
                normal_img = grab_frame(normal,i,normal_path,normalize)
                gtdepth_img = grab_frame(gtdepth,i,gtdepth_path,normalize_depth)
                depth_img = grab_frame(depth,i,depth_path,hot_vectorize)
                a = util.bounding_box(gtdepth_img,img_height)

            x1,x2,y1,y2 = a
            depth_img = depth_img[x1:x2,y1:y2] 
            gtdepth_img = gtdepth_img[x1:x2,y1:y2]
            normal_img = normal_img[x1:x2,y1:y2]
            gtdepth_img[gtdepth_img==0]=1.

            stack = stack_frames([gtdepth_img,normal_img],img_height,img_width,4)
            stack1[j] = stack
            stack2[j] = np.reshape(depth_img,(img_height,img_width,1))
        yield (stack1,stack2)

def generate_data_custom3(depth_as_mask=True,img_height=480,img_width=640,batch_size=4,path="/media/drc/DATA/CNN/",dir_name = "test/",filter_files = None,func=None):

    #rgb_path = path+"rgb/"+"rgb/"
    depth_path = path+"depth/"+ "depth/"
    gtdepth_path = path+"gtdepth/" + "gtdepth/"
    normal_path = path+"normal/" + "normal/"

    #rgb = np.sort(os.listdir(rgb_path))
    normal = np.sort(os.listdir(normal_path))
    depth = np.sort(os.listdir(depth_path))
    gtdepth = np.sort(os.listdir(gtdepth_path))

    if filter_files:
            #rgb = np.sort(filter(lambda x: filter_files in x, rgb))
            normal = np.sort(filter(lambda x: filter_files in x, normal))
            depth = np.sort(filter(lambda x: filter_files in x, depth))
            gtdepth = np.sort(filter(lambda x: filter_files in x, gtdepth))

    i = -1
    while True:
        stack1 = np.zeros((batch_size,img_height,img_width,4))
        stack2 = np.zeros((batch_size,img_height,img_width,1))

        for j in range(batch_size):
            i= (i+1)%len(depth)
            #rgb_img = grab_frame(rgb,i,rgb_path,func)
            normal_img = grab_frame(normal,i,normal_path,normalize)
            gtdepth_img = grab_frame(gtdepth,i,gtdepth_path,normalize_depth)
            depth_img = grab_frame(depth,i,depth_path)

            depth_img = depth_img[120:320,240:440] 
            gtdepth_img = gtdepth_img[120:320,240:440]  
            normal_img = normal_img[120:320,240:440]
            gtdepth_img[gtdepth_img==0]=1.

            stack = stack_frames([gtdepth_img,normal_img],img_height,img_width,4)
            stack1[j] = stack
            stack2[j] = np.reshape(depth_img,(img_height,img_width,1))
        yield (stack1,stack2)

def gen_samples(directory,shuffle = True,filter_files=None):
    samples = []
    dirs = os.listdir(directory)
    for i in dirs:
    	if filter_files and i in filter_files:
	        path = os.path.join(directory, i)+"/"
	        if os.access(path, os.R_OK):
	            gt_depth = sorted(glob.glob(path+"*_truth.png"))
	            depth = sorted(glob.glob(path+"*_depth.png"))
	            samples.extend(zip(gt_depth,depth))
    if shuffle:
        random.shuffle(samples)
    return samples                

def generate_data_custom_depth(samples,img_height=480,img_width=640,batch_size=8,mask_depth = False):
    i = 0
    while True:
        stack1 = np.zeros((batch_size,img_height,img_width,1))
        stack2 = np.zeros((batch_size,img_height,img_width,1))
        j=0
        while j<batch_size:
            try:
                rgb = samples[i][0]
                depth = samples[i][1]
                rgb_img = grab_frame1(rgb,normalize_depth)
                depth_img = grab_frame1(depth,hot_vectorize) if mask_depth else grab_frame1(depth)
                stack1[j] = np.reshape(rgb_img,(img_height,img_width,1))
                stack2[j] = np.reshape(depth_img,(img_height,img_width,1))
                j+=1
                i= (i+1)%len(samples)
            except(IOError,TypeError,ValueError):
                i= (i+1)%len(samples)
                continue
        yield (stack1,stack2)

def grab_frame1(path,func=None):
    img = misc.imread(path)
    if func:
        return func(img)
    return img