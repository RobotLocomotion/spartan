# All fields should be length either 0 or 1
# 0 indicates optional and will be ignored

float32[] joint_torque_external_l2_norm

# force threshold (applied at frame specified in message) 
# for each of the 3 cardinal directions
# Typically frame_id would be something like gripper_base_frame
# or iiwa_link_ee_gripper_aligned
Vector3Stamped[] ee_force_threshold