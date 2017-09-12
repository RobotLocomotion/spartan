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

        # Generate a list of indices of the occupied nodes, to make it
        # easier to pick an occupied node at random
        self.occupied_nodes = np.argwhere(self.occupancy)

        self.padded_occupancy = None
        self.padded_known = None
        self.padded_window_size = None

        f.close()

    def GeneratePointwiseSampleAtPoint(self, window_size, x, y, z):
        ''' Returns a voxel grid of edge length window_size
            centered at index x, y, z '''
        # Re-generate padding?
        if not self.padded_window_size or self.padded_window_size != window_size:
            self.padded_window_size = window_size
            self.padded_occupancy = np.pad(self.occupancy, window_size/2, 'constant') # Default constant is 0
            self.padded_known = np.pad(self.known, window_size/2, 'constant') # Default constant is 0

        return np.hstack([
                np.reshape(self.occupancy[x:(x+window_size), 
                           y:(y+window_size),
                           z:(z+window_size)],
                           window_size**3),
                np.reshape(self.known[x:(x+window_size), 
                           y:(y+window_size),
                           z:(z+window_size)],
                           window_size**3)
            ])

    def GenerateAllPointwiseSamplesFromVDF(self, window_size, step, min_positive_cells):
        ''' Steps over the voxel grid and spits out *all* local regions as rows in a
        large np array. This is likely to overflow memory for large voxel grids. '''
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
        print("Generating up to %d examples, stepping by %d..." % (n_examples, step))

        batch_expand_size = 100000
        examples_in = np.zeros((batch_expand_size, 
            2 * (window_size**3)), dtype=np.uint8)
        examples_out = np.zeros((batch_expand_size, 1), dtype=np.uint8)

        k = 0
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    if (k >= examples_out.shape[0]):
                            examples_in.resize( (k+batch_expand_size, window_size**3) )
                            examples_out.resize( (k+batch_expand_size, 1))

                    examples_in[k, 0:window_size**3] = np.reshape(
                        self.occupancy[x:(x+window_size), 
                                       y:(y+window_size),
                                       z:(z+window_size)],
                        window_size**3)

                    if np.sum(examples_in[k, 0:window_size**3] > 0) >= min_positive_cells:
                        examples_in[k, -window_size**3:] = np.reshape(
                            self.known[x:(x+window_size), 
                                       y:(y+window_size),
                                       z:(z+window_size)],
                            window_size**3)
                        examples_out[k] = self.occupancy[
                              floor(x+window_size/2),
                              floor(y+window_size/2),
                              floor(z+window_size/2)]
                        k+=1
                        if k % 1000 == 0:
                            print("k=%d, maxk=%d, xyz = %d,%d,%d" % (k, x*len(y_range)*len(z_range)+y*len(z_range) + z, x, y, z))

        print("\t\t\tdone.")
        examples_in.resize((k, 2*(window_size**3)))
        examples_out.resize((k, 1))
        return (examples_in, examples_out)

    def GenerateRandomPointwiseSamplesFromVDF(self, window_size, N, min_positive_cells, positive_example_ratio=-1):
        ''' Generates N random local regions as rows in an np array. (A local region is
        a reshaped window_size**3 cube of occupancy, and then a reshaped window_size**3 cube
        of known-ness.) Each known region will contain at least min_positive_cells occupied
        cells. (This is useful because most of the vdfs are typically empty space.)
        Positive_example_ratio fraction of the examples will be drawn from the set of known
        occupied cells.'''

        # These are the coordinates at which the box will be *centered*.
        # Out-of-range cells will be filled with empty, unknown grid cells.
        xmin = 0
        xmax = self.counts.shape[0]-1
        ymin = 0
        ymax = self.counts.shape[1]-1
        zmin = 0
        zmax = self.counts.shape[2]-1

        if positive_example_ratio < 0:
            # Just draw them all randomly
            num_negative = N
            print("Generating %d randomly chosen examples" % N)
        else:
            num_positive = positive_example_ratio * N
            num_negative = N - num_positive
            print("Generating %d randomly chosen examples, %d of them positive" % (N, num_positive))

        examples_in = np.zeros((N, 
            2*(window_size**3)), dtype=np.uint8)
        examples_out = np.zeros((N, 1), dtype=np.uint8)

        k = 0
        while k < num_negative:
            x = np.random.randint(xmin, xmax)
            y = np.random.randint(zmin, ymax)
            z = np.random.randint(zmin, zmax)

            examples_in[k, :window_size**3] = np.reshape(
                        self.occupancy[x:(x+window_size), 
                                       y:(y+window_size),
                                       z:(z+window_size)],
                        window_size**3)

            examples_out[k] = self.occupancy[
                            floor(x+window_size/2),
                            floor(y+window_size/2),
                            floor(z+window_size/2)]

            if num_positive > 0 and examples_out[k]:
                # If we have a ratio to meet, sample only negative cells
                # in this round.
                continue

            if np.sum(examples_in[k, :] > 0) >= min_positive_cells:
                # Grab the last bit and continue
                examples_in[k, -window_size**3:] = np.reshape(
                        self.known[x:(x+window_size), 
                                       y:(y+window_size),
                                       z:(z+window_size)],
                        window_size**3)
                k+=1
                if k % 1000 == 0:
                    print("k=%d/%d" % (k, N))

        while k < N:
            # Fill rest with positive examples
            ind = np.random.randint(0, self.occupied_nodes.shape[0])
            pt = self.occupied_nodes[ind, :]
            if     xmin <= pt[0] and pt[0] < xmax \
               and ymin <= pt[1] and pt[1] < ymax \
               and zmin <= pt[2] and pt[2] < zmax:
                examples_in[k, :] = self.GeneratePointwiseSampleAtPoint(window_size, pt[0], pt[1], pt[2])
                examples_out[k] = self.occupancy[pt[0], pt[1], pt[2]]
                if not examples_out[k]:
                    print("Examples out is false but should be true in sampling known occupied cells")
                    exit(-1)
                k+=1
                if k % 1000 == 0:
                    print("k=%d/%d" % (k, N))

        print("\t\t\tdone.")

        # Shuffle them together
        order = np.arange(examples_out.shape[0])
        np.random.shuffle(order)
        examples_in = examples_in[order, :]
        examples_out = examples_out[order, :]
        return (examples_in, examples_out)

if __name__ == "__main__":
    import sys
    vdf = VoxelDistanceFunction(sys.argv[1])
    print(vdf.size)
    print(np.sum(vdf.occupancy), vdf.occupied_nodes.shape)
    samples_in, samples_out = vdf.GenerateRandomPointwiseSamplesFromVDF(window_size=20, N=10000, min_positive_cells=10, positive_example_ratio=0.5)
    print(np.sum(samples_out), "/", samples_out.shape[0], " are positive")
