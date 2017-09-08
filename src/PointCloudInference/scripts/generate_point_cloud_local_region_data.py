import argparse
import numpy as np
import yaml
import keras

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("input")

    args = parser.parse_args()

    vdf_info = yaml.load(open(args.input), Loader=yaml.CLoader)

    size = vdf_info["size"]
    lb = vdf_info["lb"]
    ub = vdf_info["ub"]

    counts = np.frombuffer(bytearray(vdf_info["counts"]), dtype=np.uint32)
    distances = np.frombuffer(bytearray(vdf_info["distances"]), dtype=np.float64)
    known = np.frombuffer(bytearray(vdf_info["known"]), dtype=np.uint8)

    counts = np.reshape(counts, size)
    occupancy = np.greater_equal(counts, 1)
    distances = np.reshape(distances, size)
    known = np.reshape(known, size)

    WINDOW_SIZE = 30
    STEP=2

    xmin = 0
    xmax = counts.shape[0]-WINDOW_SIZE-1
    ymin = 0
    ymax = counts.shape[1]-WINDOW_SIZE-1
    zmin = 0
    zmax = counts.shape[2]-WINDOW_SIZE-1


    x_range = range(xmin, xmax, STEP)
    y_range = range(ymin, ymax, STEP)
    z_range = range(zmin, zmax, STEP)

    n_examples = len(x_range)*len(y_range)*len(z_range)
    print "Generating %d examples..." % n_examples

    examples_in = np.zeros((n_examples, 
        WINDOW_SIZE*WINDOW_SIZE*WINDOW_SIZE))
    examples_out = np.zeros((n_examples, 1))


    k = 0
    for x in x_range:
        for y in y_range:
            for z in z_range:
                examples_in[k, :] = np.reshape(
                    occupancy[x:(x+WINDOW_SIZE), 
                              y:(y+WINDOW_SIZE),
                              z:(z+WINDOW_SIZE)],
                    WINDOW_SIZE*WINDOW_SIZE*WINDOW_SIZE)
                examples_out[k] = occupancy[x, y, z]
                k+=1

