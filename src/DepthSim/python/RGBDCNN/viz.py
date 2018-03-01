import cv2
import matplotlib.pyplot as plt
import os
from scipy import misc
import numpy as np
import data
import network

def viz_nn_stream_custom(sleep =.5,path = "/media/drc/DATA/chris_labelfusion/CORL2017/test_data/"):
	img_height = 480
	img_width = 640
	model = network.load_trained_model()
	data_gen = data.generate_data_custom(img_height=480,img_width=640,batch_size=2,path=path,dir_name = "test/",filter_files = None,func=data.normalize)
	
	stack = data_gen.next()
	gtdepth,normal,rgb,depth = network.decompose_training_stack(stack,depth= True)#do depth the same way as keras

	depth = np.reshape(depth,(img_height,img_width))
	rgb = np.reshape(rgb,(img_height,img_width,3))
	gtdepth = np.reshape(gtdepth,(img_height,img_width))
	normal = np.reshape(normal,(img_height,img_width,3))


	predicted_prob_map = model.predict_on_batch(stack[:,:,:,:7])
	predicted_depth = network.apply_mask(predicted_prob_map,gtdepth)
	predicted_depth = np.reshape(predicted_depth,(img_height,img_width))

	ax1 = plt.subplot(1,3,1)
	ax2 = plt.subplot(1,3,2)
	ax3 = plt.subplot(1,3,3)

	im1 = ax1.imshow(rgb)
	im2 = ax2.imshow(np.reshape(predicted_prob_map,(img_height,img_width)))
	im3 = ax3.imshow(depth)

	plt.ion()

	while True:
		stack = data_gen.next()
		gtdepth,normal,rgb,depth = network.decompose_training_stack(stack,depth= True)

		depth = np.reshape(depth,(img_height,img_width))
		rgb = np.reshape(rgb,(img_height,img_width,3))
		gtdepth = np.reshape(gtdepth,(img_height,img_width))
		normal = np.reshape(normal,(img_height,img_width,3))


		predicted_prob_map = model.predict_on_batch(stack[:,:,:,:7])
		predicted_depth = network.apply_mask(predicted_prob_map,gtdepth)
		predicted_depth = np.reshape(predicted_depth,(img_height,img_width))

	
		im1.set_data(rgb)
		im2.set_data(np.reshape(predicted_prob_map,(img_height,img_width)))
		im3.set_data(depth)

		plt.pause(sleep)

	plt.ioff() # due to infinite loop, this gets never called.
	plt.show()

def viz_nn_stream_keras(sleep =.5,path = "/media/drc/DATA/chris_labelfusion/CORL2017/test_data/"):
	img_height = 480
	img_width = 640
	model = network.load_trained_model()
	data_gen = data.generate_data(depth_as_mask= False, batch_size = 1, path = path)
	
	test = data_gen.next()
	stack,depth = test
	gtdepth,normal,rgb = network.decompose_training_stack(stack)

	depth = np.reshape(depth,(img_height,img_width))
	rgb = np.reshape(rgb,(img_height,img_width,3))
	gtdepth = np.reshape(gtdepth,(img_height,img_width))
	normal = np.reshape(normal,(img_height,img_width,3))

	predicted_prob_map = model.predict_on_batch(stack)
	predicted_depth = network.apply_mask(predicted_prob_map,gtdepth)

	predicted_depth = np.reshape(predicted_depth,(img_height,img_width))

	plt.figure()
	plt.imshow(predicted_depth)
	plt.show()

	ax1 = plt.subplot(1,2,1)
	ax2 = plt.subplot(1,2,2)

	im1 = ax1.imshow(rgb)
	
	im2 = ax2.imshow(np.reshape(predicted_prob_map,(img_height,img_width)))

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
	
		im1.set_data(rgb)
		im2.set_data(np.reshape(predicted_prob_map,(480,640)))
		plt.pause(sleep)

	plt.ioff() # due to infinite loop, this gets never called.
	plt.show()

def viz_stream_from_dir(sleep =.01,path = "/media/drc/DATA/chris_labelfusion/CORL2017/test_data/",filter_files = None):
	img_height = 480
	img_width = 640
	model = network.load_trained_model()

	rgb_path = path+"rgb/rgb/"
	depth_path = path+"depth/depth/"
	gtdepth_path = path+"gtdepth/gtdepth/"
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
	#create two subplots
	ax1 = plt.subplot(1,3,1)
	ax2 = plt.subplot(1,3,2)
	ax3 = plt.subplot(1,3,3)

	i = 0
	rgb_img = grab_frame(rgb,i,rgb_path)
	normal_img = grab_frame(normal,i,normal_path)
	gtdepth_img = grab_frame(gtdepth,i,gtdepth_path)
	depth_img = grab_frame(depth,i,depth_path)

	stack = stack_frames(gtdepth_img,normal_img,rgb_img,img_height,img_width)

	predicted_prob_map = model.predict_on_batch(stack)

	predicted_depth = network.apply_mask(predicted_prob_map,gtdepth_img)
	#create two image plots

	im1 = ax1.imshow(rgb_img)
	im2 = ax2.imshow(np.reshape(predicted_prob_map,(480,640)))
	im3 = ax3.imshow(depth_img)

	plt.ion()

	while True:
		i=(i+1)%len(rgb)
		rgb_img = grab_frame(rgb,i,rgb_path)
		normal_img = grab_frame(normal,i,normal_path)
		gtdepth_img = grab_frame(gtdepth,i,gtdepth_path)
		depth_img = grab_frame(depth,i,depth_path)

		stack = stack_frames(gtdepth_img,normal_img,rgb_img,img_height,img_width)
		predicted_prob_map = model.predict_on_batch(stack)
		#predicted_depth = network.apply_mask(predicted_prob_map,gtdepth_img)

		im1.set_data(rgb_img)
		im2.set_data(np.reshape(predicted_prob_map,(480,640)))
		im3.set_data(depth_img)
		plt.pause(sleep)

	plt.ioff() # due to infinite loop, this gets never called.
	plt.show()

def show_prob_map_dist(img):
	plt.figure()
	plt.hist(img.ravel(), bins=256)
	plt.show()

if __name__ == '__main__':
	#viz_nn_stream()
	#viz_stream(path = "/media/drc/DATA/CNN/",filter_files = "2017-06-16-06")
	viz_nn_stream_custom()