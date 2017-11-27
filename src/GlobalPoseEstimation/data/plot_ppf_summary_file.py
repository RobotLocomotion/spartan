import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import yaml

histogram_info = yaml.load(open(sys.argv[1]))

dims = [histogram_info["n_bins"]["distance"],
		histogram_info["n_bins"]["n1_n2"],
		histogram_info["n_bins"]["d_n1"],
		histogram_info["n_bins"]["d_n2"]]

maxdist = histogram_info["max_distance"]
maxondim = [maxdist, math.pi, math.pi, math.pi]
labels = ["distance", "n1_n2", "d_n1", "d_n2"]

counts = np.reshape(histogram_info["histogram_counts"], dims, order="C").astype(float)
means = np.reshape(histogram_info["histogram_means"], dims, order="C").astype(float)
variances = np.reshape(histogram_info["histogram_variances"], dims, order="C").astype(float)

plt.figure()

plt.title("Histogram distributions in each dimension")

for i in range(4):
	axis = range(4)[0:i] + range(4)[i+1:]

	# to collapse in a direction, we need to combine the statistics
	# across the other dimensions

	# the mean is just a weighted average by sample size
	this_dimension_means = np.sum(means * counts, axis=tuple(axis)) / np.sum(counts, axis=tuple(axis))

	# following a generalization of
	# https://stats.stackexchange.com/questions/43159/how-to-calculate-pooled-variance-of-two-groups-given-known-group-variances-mean
	this_dimension_variances = np.sum( (means + variances)*counts, axis=tuple(axis)) / np.sum(counts, axis=tuple(axis))  - this_dimension_means

	plt.subplot(2, 2, i+1)
	axis = range(4)[0:i] + range(4)[i+1:]
	data_range = np.arange(dims[i])*maxondim[i]/dims[i]
	plt.plot(data_range, this_dimension_means)
	plt.fill(np.concatenate([data_range, data_range[::-1]]),
		     np.concatenate([this_dimension_means - 1.96 * this_dimension_variances,
		     	            (this_dimension_means + 1.96 * this_dimension_variances)[::-1]]),
		     alpha=0.5, fc='b', ec='None', label='95% confidence interval')
	plt.xlabel(labels[i])
	plt.ylabel("Distribution of hits")

plt.show()

