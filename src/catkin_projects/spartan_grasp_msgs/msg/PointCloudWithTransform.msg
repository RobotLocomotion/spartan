# Records a pointcloud along with the camera origin where it was captured and 
# the transform to a given base frame
Header header
sensor_msgs/PointCloud2 point_cloud
geometry_msgs/Point camera_origin # typically will be (0,0,0)
geometry_msgs/TransformStamped point_cloud_to_base_transform # pointcloud should be the child frame