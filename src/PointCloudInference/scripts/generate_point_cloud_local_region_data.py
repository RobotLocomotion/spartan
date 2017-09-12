import argparse
import numpy as np
from VoxelDistanceFunctionFileLoader import *
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("window_size")
    parser.add_argument("N")
    parser.add_argument("min_active_cells_per_window")

    args = parser.parse_args()

    vdf = VoxelDistanceFunction(args.input)

    window_size = int(args.window_size)
    N = int(args.N)
    min_active_cells_per_window = int(args.min_active_cells_per_window)
    examples_in, examples_out = vdf.GenerateRandomPointwiseSamplesFromVDF(window_size, N, min_active_cells_per_window)

    name = args.input.split("/")[-1].split(".")[0]
    in_file = "%s.in.npy" % name
    out_file = "%s.out.npy" % name
    print "Saving as: %s and %s" % (in_file, out_file)
    np.save(in_file, examples_in)
    np.save(out_file, examples_out)


