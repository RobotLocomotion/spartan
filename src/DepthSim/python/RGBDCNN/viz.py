import cv2
import matplotlib.pyplot as plt
import os
from scipy import misc
import numpy as np
import data
import network
import glob
import scipy

def viz_predicted_depth(path,model_path,sleep =.1,filter_files= None,img_height=480,img_width=640,save_dir = None): #add file filter for specific logs
	model = network.load_trained_model(weights_path = model_path)
	samples = data.gen_samples(path,False,filter_files=filter_files)
	
	print "generarting samples"
	train = data.generate_data_custom_depth(samples,batch_size = 1)
	
	stack = train.next()
	depth = np.reshape(stack[1],(img_height,img_width))
	gtdepth = np.reshape(stack[0],(img_height,img_width))

	threshold = .3
	predicted_prob_map = model.predict_on_batch(stack[0])
	network.apply_mask(predicted_prob_map,gtdepth,threshold)

	ax1 = plt.subplot(1,2,1)
	ax2 = plt.subplot(1,2,2)
	#ax3 = plt.subplot(1,3,3)

	im1 = ax1.imshow(depth)
	im2 = ax2.imshow(gtdepth*3500)
	#im3 = ax3.imshow(depth)

	plt.ion()
	
	int = 0
	while True:
		stack = train.next()
		depth = np.reshape(stack[1],(img_height,img_width))
		gtdepth = np.reshape(stack[0],(img_height,img_width))


		predicted_prob_map = model.predict_on_batch(stack[0])
		network.apply_mask(predicted_prob_map,gtdepth,threshold)
		im1.set_data(depth)
		im2.set_data(gtdepth*3500)
		#im3.set_data(depth)

		if save_dir:
			misc.imsave(save_dir)
		plt.pause(sleep)

	plt.ioff() # due to infinite loop, this gets never called.
	plt.show()

def viz_predicted_depth1(path,model_path,sleep =.1,filter_files= None,img_height=480,img_width=640,save_dir = None,viz=True): #add file filter for specific logs
	model = network.load_trained_model(weights_path = model_path)
	samples = gen_samples(path,False,filter_files=filter_files)
	stack = np.zeros((1,img_height,img_width,1))

	threshold = .5
	ax1 = plt.subplot(1,4,1)
	ax2 = plt.subplot(1,4,2)
	ax3 = plt.subplot(1,4,3)
	ax4 = plt.subplot(1,4,4)

	im1 = ax1.imshow(misc.imread(samples[0][1]))
	im2 = ax2.imshow(misc.imread(samples[0][0]))
	im3 = ax3.imshow(misc.imread(samples[0][0]))
	im4 = ax4.imshow(misc.imread(samples[0][2]))
	plt.ion()

	for i in range(len(samples)):
		rgb = misc.imread(samples[i][2])
		depth = misc.imread(samples[i][1])
		gtdepth = misc.imread(samples[i][0])/3500.
		stack[0,:,:,0] = gtdepth
		gt_copy = np.copy(gtdepth)


		predicted_prob_map = model.predict_on_batch(stack)
		network.apply_mask(predicted_prob_map,gtdepth,threshold)

		im1.set_data(depth)
		im2.set_data(gtdepth*4500)
		im3.set_data(gt_copy*4500)
		im4.set_data(rgb)

		if save_dir:

			scipy.misc.toimage(rgb).save(save_dir+str(i)+"rgb.png")
			scipy.misc.toimage(depth, cmin=0, cmax=3500,mode = "I").save(save_dir+str(i)+"depth.png")
			scipy.misc.toimage(gt_copy*3500, cmin=0, cmax=3500,mode = "I").save(save_dir+str(i)+"gtdepth.png")
			scipy.misc.toimage(gtdepth*3500, cmin=0, cmax=3500,mode = "I").save(save_dir+str(i)+"predicted_depth.png")
			# import imageio
			# imageio.imwrite(save_dir+str(i)+"rgb.png",rgb)
			# imageio.imwrite(save_dir+str(i)+"depth.png",depth)
			# imageio.imwrite(save_dir+str(i)+"gtdepth.png",gt_copy*3500.)
			# imageio.imwrite(save_dir+str(i)+"predicted_depth.png",gtdepth*3500.)

		plt.pause(sleep)

	plt.ioff() # due to infinite loop, this gets never called.
	plt.show()


def gen_samples(directory,shuffle = True,filter_files=None):
    samples = []
    dirs = os.listdir(directory)
    for i in dirs:
		if filter_files and i in filter_files:
			path = os.path.join(directory, i)+"/"
			if os.access(path, os.R_OK):
				gt_depth = sorted(glob.glob(path+"*_depth_*"))
				depth = sorted(glob.glob(path+"*_depth.png"))
				rgb = sorted(glob.glob(path+"*rgb.png"))
				samples.extend(zip(gt_depth,depth,rgb))
    if shuffle:
        random.shuffle(samples)
    return samples 

def show_prob_map_dist(img):
	plt.figure()
	plt.hist(img.ravel(), bins=256)
	plt.show()

if __name__ == '__main__':
	save_dir = "/media/drc/DATA/chris_labelfusion/RGBDCNNTest/"
	viz_predicted_depth1(sleep =.05,filter_files = " 2017-06-16-19",path = "/media/drc/DATA/chris_labelfusion/RGBDCNN/",model_path = "../models/net_depth_seg_v1.hdf5")#,save_dir = save_dir)