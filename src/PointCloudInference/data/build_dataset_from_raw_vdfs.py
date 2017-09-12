import argparse
from math import ceil
import numpy as np
import os
import random
from VoxelDistanceFunctionFileLoader import *

def generate_random_samples_from_vdf(filename, output_prefix, window_size, N_samples, min_active_cells_per_window, positive_ratio, N_batches):
    vdf = VoxelDistanceFunction(filename)

    for k in range(N_batches):
        examples_in, examples_out = vdf.GenerateRandomPointwiseSamplesFromVDF(window_size, N_samples, min_active_cells_per_window, positive_ratio)

        in_file = "%s_%d.in.npy" % (output_prefix, k)
        out_file = "%s_%d.out.npy" % (output_prefix, k)
        print "Saving as: %s and %s" % (in_file, out_file)
        np.save(in_file, examples_in, allow_pickle=False)
        np.save(out_file, examples_out, allow_pickle=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("window_size")
    parser.add_argument("N")
    parser.add_argument("min_active_cells_per_window")
    parser.add_argument("positive_ratio")
    parser.add_argument("N_batches")
    parser.add_argument("percent_test")

    args = parser.parse_args()

    window_size = int(args.window_size)
    N = int(args.N)
    min_active_cells_per_window = int(args.min_active_cells_per_window)
    positive_ratio = float(args.positive_ratio)
    N_batches = int(args.N_batches)
    percent_test = float(args.percent_test)

    all_inputs = next(os.walk("./raw_vdfs"))[2]

    good_inputs = []
    for input_name in all_inputs:
        filename_split = os.path.splitext(input_name)
        if filename_split[1] == ".vdf":
            good_inputs.append(input_name)

    # Split files into test and train clouds
    num_test = int(ceil(percent_test * len(good_inputs)))
    random.shuffle(good_inputs)
    test_inputs = good_inputs[:num_test]
    train_inputs = good_inputs[num_test+1:]
    print test_inputs, train_inputs


    os.system("mkdir -p train")
    for input_name in train_inputs:
        generate_random_samples_from_vdf("./raw_vdfs/%s" % input_name, 
            "train/" + os.path.splitext(input_name)[0],
            window_size,
            N,
            min_active_cells_per_window,
            positive_ratio, N_batches)


    os.system("mkdir -p test")
    for input_name in test_inputs:
        generate_random_samples_from_vdf("./raw_vdfs/%s" % input_name, 
            "test/" + os.path.splitext(input_name)[0],
            window_size,
            N,
            min_active_cells_per_window,
            positive_ratio, N_batches)


