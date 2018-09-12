## Grasp Generation CNN
Documentation related to the integration of the GGCNN from [this](https://github.com/dougsm/ggcnn) paper into `spartan`. 

### Network Input & Output

Taken directly from Doug's code

```
Input: Inpainted depth, subtracted mean, in meters, with random rotations and zoom. 
Output: q, cos(2theta), sin(2theta), grasp_width in pixels/150.
```

#### Input
Depth image, expressed in meters. It be inpainted using OpenCV. Zero mean by subtracting mean. Scaled to [-1,1]. See 
[here](https://github.com/dougsm/ggcnn_kinova_grasping/blob/master/ggcnn_kinova_grasping/scripts/run_ggcnn.py#L92)

##### Output
