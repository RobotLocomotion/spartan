from math import ceil
import numpy as np
import re
import os
import sys

class DataGenerator(object):
  'Generates data for Keras'
  def __init__(self, data_dir = "", window_size = 20, batch_size = 32):
      'Initialization'
      self.data_dir = data_dir
      self.window_size = window_size
      self.batch_size = batch_size

  def generate(self):
      'Generates batches of samples from the given directory'

      INPUT_FILTER = re.compile(".*.in.npy")
      raw_filenames = [x for x in next(os.walk(self.data_dir))[2] if INPUT_FILTER.match(x)]

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
              X = input_buffer[start_ind:end_ind, :]
              y = output_buffer[start_ind:end_ind]

              yield X, y


if __name__ == "__main__":
  # Test on whatever gets passed in
  dg = DataGenerator(sys.argv[1], 20, 32)
  g = dg.generate()
  for i in range(10):
    print(next(g))