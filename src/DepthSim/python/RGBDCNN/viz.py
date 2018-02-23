import cv2
import matplotlib.pyplot as plt
import os
from scipy import misc
import numpy as np
import data
import network


def viz_stream(path = "/media/drc/DATA/chris_labelfusion/CORL2017/test_data/" ,sleep =.1):
	def grab_frame(files,i,path):
	    img = misc.imread(path+files[i])
	    return img
	files1 = np.sort(os.listdir("path depth/"))
	files2 = np.sort(os.listdir("/media/drc/DATA/chris_labelfusion/CORL2017/object_database/normal/"))
	#create two subplots
	ax1 = plt.subplot(1,2,1)
	ax2 = plt.subplot(1,2,2)
	i = 0

	#create two image plots
	im1 = ax1.imshow(grab_frame(files1,i,path1))
	im2 = ax2.imshow(grab_frame(files2,i,path2))

	plt.ion()

	while True:
	    i=(i+1)%len(files1)
	    im1.set_data(grab_frame(files1,i,path1))
	    im2.set_data(grab_frame(files2,i,path2))
	    plt.pause(sleep)

	plt.ioff() # due to infinite loop, this gets never called.
	plt.show()

def viz_nn_stream(sleep =.5,path = "/media/drc/DATA/chris_labelfusion/CORL2017/test_data/"):
	img_height = 480
	img_width = 640
	model = network.load_trained_model()
	data_gen = data.generate_data(depth_as_mask= False, batch_size = 1, rgb=path+"rgb",depth=path+"depth",normal=path+"normal",gtdepth=path+"gtdepth")
	
	test = data_gen.next()
	stack,depth = test
	gtdepth,normal,rgb = network.decompose_training_stack(stack)

	depth = np.reshape(depth,(img_height,img_width))
	rgb = np.reshape(rgb,(img_height,img_width,3))
	gtdepth = np.reshape(gtdepth,(img_height,img_width))
	normal = np.reshape(normal,(img_height,img_width,3))

	predicted_prob_map = model.predict_on_batch(stack)
	predicted_depth = network.apply_mask(predicted_prob_map,depth)

	
	predicted_depth = np.reshape(predicted_depth,(img_height,img_width))

	ax1 = plt.subplot(1,2,1)
	ax2 = plt.subplot(1,2,2)

	im1 = ax1.imshow(rgb)
	im2 = ax2.imshow(predicted_depth)

	plt.ion()

	while True:
		test = data_gen.next()
		stack,depth = test
		gtdepth,normal,rgb = network.decompose_training_stack(stack)
		predicted_prob_map = model.predict_on_batch(stack)

		depth = np.reshape(depth,(img_height,img_width))
		rgb = np.reshape(rgb,(img_height,img_width,3))
		gtdepth = np.reshape(gtdepth,(img_height,img_width))
		normal = np.reshape(normal,(img_height,img_width,3))

		predicted_depth = network.apply_mask(predicted_prob_map,depth)

		
		predicted_depth = np.reshape(predicted_depth,(img_height,img_width))
	
		im1 = ax1.imshow(rgb)
		im2 = ax2.imshow(predicted_depth)
		plt.pause(sleep)

	plt.ioff() # due to infinite loop, this gets never called.
	plt.show()

def show_prob_map_dist(img):
	plt.figure()
	plt.hist(img.ravel(), bins=256)
	plt.show()

if __name__ == '__main__':
	viz_nn_stream(path = "/media/drc/DATA/CNN/")