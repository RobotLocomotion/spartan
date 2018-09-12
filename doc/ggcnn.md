## Grasp Generation CNN
Documentation related to the integration of the GGCNN from [this](https://github.com/dougsm/ggcnn) paper into `spartan`. 

### Network Input & Output

Taken directly from Doug's code

```
Input: Inpainted depth, subtracted mean, in meters, with random rotations and zoom. 
Output: q, cos(2theta), sin(2theta), grasp_width in pixels/150.
```

#### Input
Depth image, expressed in meters. Cropped to be `300 x 300`. It is inpainted using OpenCV. Zero mean by subtracting mean. Clipped to [-1,1]. See [here](https://github.com/dougsm/ggcnn_kinova_grasping/blob/master/ggcnn_kinova_grasping/scripts/run_ggcnn.py#L92)

##### Output
The output is a list with 4 elements: `q, cos(2theta), sin(2theta), grasp_width in pixels/150.` We can recover the angle `theta` from `cos(2theta), sin(2theta)`. To smooth things out a guassian filter is applied to both `q, theta`.

- **q:** Numpy array with shape `[300,300]`. Contains the score for each grasp location.
- **angle:** Numpy array with shape `[300, 300]`. Contains the top predicted angle for each pixel location.
- **width** `grasp_width/150`. Note that this is in the weird crop/resize thing

Now you need to convert the max pixel back to uncropped/resized image coordinates so we can do the camera transform.
