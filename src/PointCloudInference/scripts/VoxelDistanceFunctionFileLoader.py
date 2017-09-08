import argparse
from math import floor
import numpy as np
import yaml

class VoxelDistanceFunction:
    def __init__(self, filename):
        f = open(filename)
        vdf_info = yaml.load(f, Loader=yaml.CLoader)

        self.size = vdf_info["size"]
        self.lb = np.array(vdf_info["lb"])
        self.ub = np.array(vdf_info["ub"])

        self.counts = np.frombuffer(bytearray(vdf_info["counts"]), dtype=np.uint32)
        self.distances = np.frombuffer(bytearray(vdf_info["distances"]), dtype=np.float64)
        self.known = np.frombuffer(bytearray(vdf_info["known"]), dtype=np.uint8)

        self.counts = np.reshape(self.counts, self.size)
        self.occupancy = np.greater_equal(self.counts, 1)
        self.distances = np.reshape(self.distances, self.size)
        self.known = np.reshape(self.known, self.size) 

        f.close()

    def GenerateOccupancySamples(self, window_size, step):
        xmin = 0
        xmax = self.counts.shape[0]-window_size-1
        ymin = 0
        ymax = self.counts.shape[1]-window_size-1
        zmin = 0
        zmax = self.counts.shape[2]-window_size-1

        x_range = range(xmin, xmax, step)
        y_range = range(ymin, ymax, step)
        z_range = range(zmin, zmax, step)

        n_examples = len(x_range)*len(y_range)*len(z_range)
        print("Generating %d examples..." % n_examples)

        examples_in = np.zeros((n_examples, 
            window_size*window_size*window_size))
        examples_out = np.zeros((n_examples, 1))

        k = 0
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    examples_in[k, :] = np.reshape(
                        self.occupancy[x:(x+window_size), 
                                       y:(y+window_size),
                                       z:(z+window_size)],
                        window_size*window_size*window_size)
                    examples_out[k] = self.occupancy[
                          floor(x+window_size/2),
                          floor(y+window_size/2),
                          floor(z+window_size/2)]
                    k+=1

        print("\t\t\tdone.")
        return (examples_in, examples_out)
    

