import numpy as np
import spartan.utils.transformations as transformations

pos = np.array([ 1.49324389e-05, -1.06322054e-05,  1.50987720e-01])
quat = np.array([ 9.99999999e-01,  2.02723118e-05,  2.95049711e-05, -8.16280282e-06])

# transform from command frame to iiwa_link_ee
# the command frame is aligned with wsg_50_base_link shifted to be at the
# center of the fingertips
T_E_cmd = transformations.quaternion_matrix(quat)
T_E_cmd[:3, 3] = pos

T_cmd_E = np.linalg.inv(T_E_cmd)