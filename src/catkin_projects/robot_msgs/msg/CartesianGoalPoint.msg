geometry_msgs/PointStamped xyz_point
geometry_msgs/Point xyz_d_point # feed-forward velocity
geometry_msgs/Quaternion quaternion

robot_msgs/CartesianGain gain

# string describing frame on the robot to use
string ee_frame_id

# alternatively can command in EE velocity mode
bool use_end_effector_velocity_mode
geometry_msgs/Vector3Stamped linear_velocity
geometry_msgs/Vector3Stamped angular_velocity

# this is just for redundant logging (to preserve
# correct rpy)
float64 roll
float64 pitch
float64 yaw