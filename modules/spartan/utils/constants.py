import numpy as np
import spartan.utils.transformations as transformations

pos = np.array([ 0.15012376, -0.0003506 , -0.00126868])
quat = np.array([0.58659034, 0.39440466, 0.58897855, 0.39174098])

# transform from command frame to iiwa_link_ee
# the command frame is aligned with wsg_50_base_link shifted to be at the
# center of the fingertips
T_E_cmd = transformations.quaternion_matrix(quat)
T_E_cmd[:3, 3] = pos

T_cmd_E = np.linalg.inv(T_E_cmd)