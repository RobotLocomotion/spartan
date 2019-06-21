std_msgs/Header header

# incoming message that we commanded against
CartesianGoalPoint cartesian_goal_point_cmd 

# the values that were really commanded
float64[] q_measured
float64[] q_cmd_prev
float64[] v_cmd
float64[] v_measured
float64[] q_cmd

geometry_msgs/Vector3 linear_velocity_cmd_expressed_in_world
geometry_msgs/Vector3 angular_velocity_cmd_expressed_in_world

geometry_msgs/Twist ee_twist_cmd_expressed_in_body

geometry_msgs/TransformStamped end_effector_to_world
geometry_msgs/TransformStamped end_effector_to_world_w_q_cmd