duration[] time_from_start # knot point time
geometry_msgs/PointStamped[] xyz_points # xyz points in world frame

# Must have length 0 or 1
# if empty, use current EE orientation
# otherwise slerp to new desired orientation
geometry_msgs/Quaternion[] quaternions

# string describing frame on the robot to use
string ee_frame_id