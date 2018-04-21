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
import noise
from scipy.misc import toimage



def create_model_2(img_height=480, img_width=640,channels=1):
   inputs = Input((img_height, img_width,channels))
   #crop = Cropping2D(cropping=((0, 0), (0, 0)), data_format=None)
   conv1 = Conv2D(4, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(inputs)
   conv1 = Conv2D(4, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv1)
   pool1 = MaxPooling2D(pool_size=(2, 2),data_format='channels_last')(conv1)

   conv2 = Conv2D(8, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(pool1)
   conv2 = Conv2D(8, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv2)
   pool2 = MaxPooling2D(pool_size=(2, 2),data_format='channels_last')(conv2)

   conv3 = Conv2D(32, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(pool2)
   conv3 = Conv2D(32, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv3)
   pool3 = MaxPooling2D(pool_size=(2, 2),data_format='channels_last')(conv3)


   conv5 = Conv2D(64, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(pool3)
   conv5 = Conv2D(64, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv5)
   drop5 = Dropout(0.5)(conv5)

   up7 = Conv2D(32, 2, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(UpSampling2D(size = (2,2),data_format='channels_last')(conv5))
   merge7 = merge([conv3,up7], mode = 'concat', concat_axis = 3)
   conv7 = Conv2D(32, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(merge7)
   conv7 = Conv2D(32, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv7)

   up8 = Conv2D(8, 2, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(UpSampling2D(size = (2,2),data_format='channels_last')(conv7))
   merge8 = merge([conv2,up8], mode = 'concat', concat_axis = 3)
   conv8 = Conv2D(8, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(merge8)
   conv8 = Conv2D(8, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv8)

   up9 = Conv2D(4, 2, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(UpSampling2D(size = (2,2),data_format='channels_last')(conv8))
   merge9 = merge([conv1,up9], mode = 'concat', concat_axis = 3)
   conv9 = Conv2D(4, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(merge9)
   conv9 = Conv2D(4, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv9)
   conv9 = Conv2D(2, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv9)
   conv10 = Conv2D(1, 1, activation = 'sigmoid',data_format='channels_last')(conv9)

   model = Model(input = inputs, output = conv10)
   print model.summary()

   return model

def create_model(img_height=480, img_width=640):
   #crop = Cropping2D(cropping=((0, 0), (0, 0)), data_format=None)
   inputs = Input((img_height, img_width,7))
  #crop = Cropping2D(cropping=((0, 0), (0, 0)), data_format=None) #check with updated param in the middle weird???
   conv1 = Conv2D(4, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(inputs)
   conv1 = Conv2D(4, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv1)
   pool1 = MaxPooling2D(pool_size=(2, 2),data_format='channels_last')(conv1)

   conv2 = Conv2D(14, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(pool1)
   conv2 = Conv2D(14, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv2)
   pool2 = MaxPooling2D(pool_size=(2, 2),data_format='channels_last')(conv2)

   conv3 = Conv2D(28, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(pool2)
   conv3 = Conv2D(28, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv3)
   pool3 = MaxPooling2D(pool_size=(2, 2),data_format='channels_last')(conv3)

   conv4 = Conv2D(56, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(pool3)
   conv4 = Conv2D(56, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv4)
   drop4 = Dropout(0.5)(conv4)
   pool4 = MaxPooling2D(pool_size=(2, 2),data_format='channels_last')(drop4)

   conv5 = Conv2D(136, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(pool4)
   conv5 = Conv2D(136, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv5)
   drop5 = Dropout(0.5)(conv5)

   up6 = Conv2D(112, 2, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(UpSampling2D(size = (2,2),data_format='channels_last')(drop5))
   merge6 = merge([drop4,up6], mode = 'concat', concat_axis = 3)
   conv6 = Conv2D(112, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(merge6)
   conv6 = Conv2D(112, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv6)

   up7 = Conv2D(56, 2, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(UpSampling2D(size = (2,2),data_format='channels_last')(conv6))
   merge7 = merge([conv3,up7], mode = 'concat', concat_axis = 3)
   conv7 = Conv2D(56, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(merge7)
   conv7 = Conv2D(56, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv7)

   up8 = Conv2D(28, 2, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(UpSampling2D(size = (2,2),data_format='channels_last')(conv7))
   merge8 = merge([conv2,up8], mode = 'concat', concat_axis = 3)
   conv8 = Conv2D(28, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(merge8)
   conv8 = Conv2D(28, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv8)

   up9 = Conv2D(14, 2, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(UpSampling2D(size = (2,2),data_format='channels_last')(conv8))
   merge9 = merge([conv1,up9], mode = 'concat', concat_axis = 3)
   conv9 = Conv2D(14, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(merge9)
   conv9 = Conv2D(14, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv9)
   conv9 = Conv2D(2, 3, activation = 'relu', padding = 'same', kernel_initializer = 'he_normal',data_format='channels_last')(conv9)
   conv10 = Conv2D(1, 1, activation = 'sigmoid',data_format='channels_last')(conv9)

   model = Model(input = inputs, output = conv10)
   return model

def train():
   model = create_model()
   print model.summary()
   model.compile(optimizer = optimizers.Adam(lr = 1e-4), loss = 'binary_crossentropy', metrics = ['accuracy'])
   model_checkpoint = ModelCheckpoint('unet1.hdf5', monitor='loss',verbose=1, save_best_only=True)
   print('Fitting model...')
   model.fit_generator(train_generator, nb_epoch=100,steps_per_epoch=100, verbose=1, shuffle=True, callbacks=[model_checkpoint])

def load_trained_model(weights_path="unet.hdf5"):
   model = create_model_2(channels=1)
   model.load_weights(weights_path)
   return model

def apply_mask(mask,depth,threshold):
   epsilon = .05
   h,w = np.shape(depth)
   mask = np.reshape(mask,(h,w))
   # plt.figure()
   # plt.imshow(mask)
   # plt.show()
   depth[mask>threshold]=0
   #img = np.random.random((480,640))
   img = sigmoid(perlin_map(scale=20.0,octaves = 7,base= np.random.randint(1000),lacunarity = 6.0))
   stochastic_mask = mask>=img
   #stochastic_mask = np.logical_and(np.logical_and((mask<=threshold), mask>epsilon), img<threshold)
   depth[stochastic_mask] = 0

def perlin_map(shape = (480,640),scale = 10.0,octaves = 6,persistence = 0.5,lacunarity = 2.0,base = 0):
    img = np.zeros(shape)
    for i in range(shape[0]):
        for j in range(shape[1]):
            img[i][j] = noise.pnoise2(i/scale, 
                                       j/scale, 
                                       octaves=octaves, 
                                       persistence=persistence, 
                                       lacunarity=lacunarity, 
                                       repeatx=shape[1], 
                                       repeaty=shape[0], 
                                       base=base)
    return img

def sigmoid(x):
    return 1. / (1 + np.exp(-10*x))

def threshold_mask(mask,threshold):
   l,h,w,r = np.shape(mask)
   mask = np.reshape(mask,(h,w))
   #mask[mask>threshold]=1
   mask[mask<threshold]=0

def prob_map_to_mask(prob_map,width=2):
    prob_map = np.copy(prob_map)
    h,w = np.shape(prob_map)
    zeros = np.argwhere(prob_map>0)
    for index in zeros:
        i = index[0]
        j = index[1]
        if i-width>0 and i+width<h and j-width>0 and j+width<w:
            if prob_map[i-width,j]<.1 or prob_map[i,j-width]<.1 or prob_map[i+width,j]<.1 or prob_map[i,j+width]<.1:
                prob_map[i,j] = np.random.binomial(1,prob_map[i,j])
            else: prob_map[i,j] = 1
    return prob_map

def prob_map_to_mask(prob_map,dev=.1):# noisify and sample from prob map #threshold for probability sample otherwise detierministic as well as location based sharp cutffs
    def sample(i):
        return np.random.binomial(1,i)
    func = np.vectorize(sample)
    return func(prob_map) 

def prob_map_to_mask(prob_map,dev=.1):
    prob_map = np.copy(prob_map)
    prob_map[prob_map>.5] = 1
    return prob_map

def decompose_training_stack(stack,depth = False,rgb = True):
   gtdpeth_img = stack[:,:,:,0]
   normal_img = stack[:,:,:,1:4]
   rgb_img = None
   depth_img = None
   if rgb:
      rgb_img = stack[:,:,:,4:7]
   if depth:
      depth_img = stack[:,:,:,7]

   return gtdpeth_img,normal_img,rgb_img,depth_img



