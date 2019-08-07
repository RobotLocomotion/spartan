import numpy as np
import spartan.utils.transformations as transformations


# transform from command frame to end-effector frame


# transform from command frame to iiwa_link_ee
# the command frame is aligned with wsg_50_base_link shifted to be at the
# center of the fingertips
pos = np.array([ 0.15012376, -0.0003506 , -0.00126868])
quat = np.array([0.58659034, 0.39440466, 0.58897855, 0.39174098])
T_E_cmd = transformations.quaternion_matrix(quat)
# T_E_cmd[:3, 3] = pos

# T_E_cmd = np.eye(4)
T_cmd_E = np.linalg.inv(T_E_cmd)



# above table pre grasp
# pos = np.array([0.51151917, 0.01527382, 0.50183972])
# quat = np.array([ 0.68753504,  0.15386359,  0.69881696, -0.12359783])
# T_W_E_init = transformations.quaternion_matrix(quat)
# T_W_E_init[:3, 3] = pos


# carrot flip
# T_W_E_init = np.asarray([[-0.00666542,  0.38384232,  0.9233746,   0.7925802 ],
#  [ 0.01898259,  0.92327729, -0.38366484, -0.01592885],
#  [-0.9997976,   0.01497076, -0.01344035,  0.26536946],
#  [ 0.,          0.,          0.,          1.        ]])
T_W_E_init = np.asarray([[-0.00714052,  0.38309528,  0.92368123,  0.7636582 ],
 [ 0.02952701,  0.92338278, -0.38274324, -0.01763667],
 [-0.99953848,  0.02454056, -0.01790509,  0.23705875],
 [ 0.        ,  0.        ,  0.        ,  1.        ]])


T_W_cmd_init = np.matmul(T_W_E_init, T_E_cmd)

# nominal box pose
pos = [0.61, 0.0, 0.05]
rpy = [0.0, 0.0, np.pi/2.0]
T_W_B_init = transformations.euler_matrix(rpy[0], rpy[1], rpy[2])
T_W_B_init[:3, 3] = pos