# the frame of the force direction is the frame in which this force_direction is expressed
# typically this would be either "base" or body_frame

# the frames must be in the urdf used by plan_runner
geometry_msgs/Vector3Stamped force
string body_frame # body frame at which this force is applied