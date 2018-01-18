import unittest
import numpy as np
import pydrake
import pydrake.rbtree
import RemoteTreeViewerWrapper_pybind as Rtv
import os.path

if __name__ == '__main__':
  r = pydrake.rbtree.RigidBodyTree(os.path.join(pydrake.getDrakePath(),
                                         "examples/pendulum/Pendulum.urdf"))
  q = r.getRandomConfiguration()
  
  rtv = Rtv.RemoteTreeViewerWrapper()
  rtv.publishRigidBodyTree(r, q, np.array([1.0, 0.0, 1.0, 1.0]), ["hello_world", "test"], True)

    
