from copy import deepcopy
from math import ceil, floor
import numpy as np
import re
import threading
import os
import sys

def sample_spherical(npoints, ndim=3):
    vec = np.random.randn(ndim, npoints)
    vec /= np.linalg.norm(vec, axis=0)
    return vec

# generators in multi-threaded applications is not thread-safe. Hence below:
class threadsafe_iter:
    """Takes an iterator/generator and makes it thread-safe by
    serializing call to the `next` method of given iterator/generator.
    """
    def __init__(self, it):
        self.it = it
        self.lock = threading.Lock()

    def __iter__(self):
        return self
    
    def __next__(self):
        with self.lock:
            return self.it.__next__()
        
def threadsafe_generator(f):
    """A decorator that takes a generator function and makes it thread-safe.
    """
    def g(*a, **kw):
        return threadsafe_iter(f(*a, **kw))
    return g

class DataGenerator(object):
  'Generates data for Keras'
  def __init__(self, data_dir = "", window_size = 20, batch_size = 32, num_cutting_planes=1):
      'Initialization'
      self.data_dir = data_dir
      self.window_size = window_size
      self.batch_size = batch_size
      self.num_cutting_planes = num_cutting_planes

  def GetWindowPadding(self, window_size):
      return (int(floor((window_size-1)/2)), int(floor(window_size/2)))
      
  @threadsafe_generator
  def generate(self):
      'Generates batches of samples from the given directory'

      INPUT_FILTER = re.compile(".*.in.npy")
      raw_filenames = [x for x in next(os.walk(self.data_dir))[2] if INPUT_FILTER.match(x)]

      # Generate a set of cutting planes we'll use for data augmentation
      num_cached_cutting_planes = 50
      cached_cutting_planes = []
      padding = self.GetWindowPadding(self.window_size)
      xx, yy, zz = np.meshgrid( range(-padding[0],padding[1]+1), 
        range(-padding[0],padding[1]+1), range(-padding[0],padding[1]+1) )
      xx = np.reshape(xx, (1, self.window_size**3))
      yy = np.reshape(yy, (1, self.window_size**3))
      zz = np.reshape(zz, (1, self.window_size**3))

      for i in range(num_cached_cutting_planes):
        normal = sample_spherical(1, 3)
        cutting_plane = (xx * normal[0] + yy * normal[1] + zz*normal[2] >= 0.)
        cached_cutting_planes.append(cutting_plane)

      # Infinite loop
      while 1:
          # Generate order of exploration of examples
          indices = np.arange(len(raw_filenames))
          np.random.shuffle(indices)

          for i in indices:
            input_name = self.data_dir + "/" + raw_filenames[i]
            output_name = self.data_dir + "/" + raw_filenames[i].split(".")[0] + ".out.npy"

            input_buffer = np.load(input_name)
            output_buffer = np.load(output_name)

            num_examples = input_buffer.shape[0]

            for i in range(int(ceil(num_examples / self.batch_size))):
              start_ind = i*self.batch_size
              end_ind = min((i+1)*self.batch_size, num_examples-1)
              X_orig = input_buffer[start_ind:end_ind, :]
              y_orig = output_buffer[start_ind:end_ind]

              X_out = deepcopy(X_orig)
              y_out = deepcopy(y_orig)

              # And append versions of X and y cut by randomly chosen
              # planes through the center, where everything on one side
              # of the plane is zero'd, to generate partial views.
              for k in range(self.num_cutting_planes):
                X_this = deepcopy(X_orig)
                y_this = y_orig
                for l in range(y_orig.shape[0]):
                  plane_i = np.random.randint(0, len(cached_cutting_planes))
                  X_this[l, np.hstack((cached_cutting_planes[plane_i], cached_cutting_planes[plane_i]))[0, :]] = 0
                X_out = np.vstack((X_out, X_this))
                y_out = np.vstack((y_out, y_this))

              yield X_out, y_out


if __name__ == "__main__":
  # Test on whatever gets passed in
  dg = DataGenerator(sys.argv[1], 20, 32)
  g = dg.generate()
  for i in range(10):
    print(next(g))