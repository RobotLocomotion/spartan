import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy import misc
import numpy as np
from keras import applications
from keras import layers
from keras import models
from keras import optimizers
from keras.preprocessing import image
import numpy as np
import tensorflow as tf

from dlutils import plot_image_batch_w_labels

#from utils.image_history_buffer import ImageHistoryBuffer


def get_images_short(num,max_digits,path):
    depth_suffix = "_depth.png"
    norm_suffix = "normal_ground_truth.png"
    ground_truth_suffix = "depth_ground_truth.png"
    prefix = str(num).zfill(max_digits)
    img_d=misc.imread(path+prefix+depth_suffix)
    img_n=misc.imread(path+prefix+norm_suffix)
    img_gt=misc.imread(path+prefix+ground_truth_suffix)
    return [img_d,img_n,img_gt]

def get_images(num,max_digits,path):
    depth_suffix = "_depth.png"
    color_suffix = "_rgb.png"
    reflec_suffix = "_rgb_r.png"
    shade_suffix = "_rgb_s.png"
    norm_suffix = "normal_ground_truth.png"
    ground_truth_suffix = "depth_ground_truth.png"
    prefix = str(num).zfill(max_digits)
    img_d=misc.imread(path+prefix+depth_suffix)
    img_c=misc.imread(path+prefix+color_suffix)
    img_s=misc.imread(path+prefix+shade_suffix)
    img_r=misc.imread(path+prefix+reflec_suffix)
    img_n=misc.imread(path+prefix+norm_suffix)
    img_gt=misc.imread(path+prefix+ground_truth_suffix)
    return [img_d,img_c,img_s,img_r,img_n,img_gt]

def create_depth_mask(img,d_low,d_high):
    mask = np.logical_and(img >= d_low,img<=d_high)
    return mask

def angle_from_normal(img):
    return np.arccos(img[:,:,2])*180/np.pi*2

def ratio_from_normal(img):
    return img[:,:,2]

def convert_rgb_normal(img):
    return (img/255.*2)-1.

def get_image_batch(generator):
        """keras generators may generate an incomplete batch for the last batch"""
        img_batch = generator.next()
        if len(img_batch) != 1:
            img_batch = generator.next()

        assert len(img_batch) == 1

        return img_batch

def bounding_box(img,size = 100):
    h,w = np.shape(img)
    non_zeros = np.nonzero(img)
    x_min = np.min(non_zeros[0])
    x_max = np.max(non_zeros[0])
    y_min = np.min(non_zeros[1])
    y_max = np.max(non_zeros[1])
    out = (x_min,x_min+size,y_min,y_min+size) if size else (x_min,x_max,y_min,y_max)#minuce or plus coordinates
    if x_min< 0 or x_min+size > h or y_min<0 or y_min+size>w:
        return None
    return out