geometry_msgs/PointStamped xyz_point
geometry_msgs/Point xyz_d_point
geometry_msgs/Quaternion quaternion

robot_msgs/CartesianGain gain

# string describing frame on the robot to use
string ee_frame_id

# this is just for redundant logging (to preserve
# correct rpy)
float64 roll
float64 pitch
float64 yaw