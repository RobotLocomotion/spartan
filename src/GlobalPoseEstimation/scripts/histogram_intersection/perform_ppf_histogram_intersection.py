import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import yaml

TEMPLATE_HISTOGRAM = "drill_decimated_mesh_ppf_histogram.yaml"
BASELINE_HISTOGRAM = "all_ppf_histogram.yaml"
SCENE_HISTOGRAM_1 = "scene_cloud_uncropped_ppf_histogram_object_neighborhood.yaml"
SCENE_HISTOGRAM_2 = "scene_cloud_uncropped_ppf_histogram_wrong_object_neighborhood.yaml"

histogram_filenames = {
    "template": TEMPLATE_HISTOGRAM,
    "baseline": BASELINE_HISTOGRAM, 
    "scene": SCENE_HISTOGRAM_1,
    "scene_2": SCENE_HISTOGRAM_2
}

histogram_data = {}

for name in histogram_filenames.keys():
    histogram_data[name] = yaml.load(open(histogram_filenames[name]))

first_name = histogram_data.keys()[0]

# Extract statistics shared across these datasets.
dims = [
        histogram_data[first_name]["n_bins"]["distance"],
        histogram_data[first_name]["n_bins"]["n1_n2"],
        histogram_data[first_name]["n_bins"]["d_n1"],
        histogram_data[first_name]["n_bins"]["d_n2"]
    ]
maxdist = histogram_data[first_name]["max_distance"]

# Extract the actual data (and sanity-check data size)
histograms = {}
for name in histogram_data.keys():
    this_dims = [
            histogram_data[name]["n_bins"]["distance"],
            histogram_data[name]["n_bins"]["n1_n2"],
            histogram_data[name]["n_bins"]["d_n1"],
            histogram_data[name]["n_bins"]["d_n2"]
        ]
    this_maxdist = histogram_data[name]["max_distance"]    
    if this_dims != dims or this_maxdist != maxdist:
        print "Dims or maxdist don't match between input files!"
        exit(0)
    histograms[name] = np.reshape(histogram_data[name]["histogram"], dims, order="C")


maxondim = [maxdist, math.pi, math.pi, math.pi]
labels = ["distance", "n1_n2", "d_n1", "d_n2"]

plt.figure()

# Show all three base histograms
dim_ind = 1
ind_range = tuple(range(4)[0:dim_ind] + range(4)[dim_ind+1:])
domain = np.arange(dims[dim_ind])*maxondim[dim_ind]/dims[dim_ind]

plt.subplot(4, 2, 1)
plt.ylabel("baseline")
baseline_summed = np.sum(histograms["baseline"], axis=(ind_range)).astype(float)
baseline_summed /= np.sum(baseline_summed)
plt.plot(domain, baseline_summed)
plt.subplot(4, 2, 2)
baseline_summed = -np.log(baseline_summed)
plt.plot(domain, baseline_summed)

plt.subplot(4, 2, 3)
plt.ylabel("template")
template_summed = np.sum(histograms["template"], axis=(ind_range)).astype(float)
template_summed /= np.sum(template_summed)
plt.plot(domain, template_summed)
plt.ylim(0, 1.0)
plt.subplot(4, 2, 4)
template_summed *= baseline_summed
template_summed /= np.sum(template_summed)
plt.plot(domain, template_summed)
plt.ylim(0, 1.0)

plt.subplot(4, 2, 5)
plt.ylabel("correct object scene")
scene_summed = np.sum(histograms["scene"], axis=(ind_range)).astype(float)
scene_summed /= np.sum(scene_summed)
plt.plot(domain, scene_summed)
plt.ylim(0, 1.0)
plt.subplot(4, 2, 6)
scene_summed *= baseline_summed
scene_summed /= np.sum(scene_summed)
plt.plot(domain, scene_summed)
plt.ylim(0, 1.0)

plt.subplot(4, 2, 7)
plt.ylabel("wrong object scene")
scene_2_summed = np.sum(histograms["scene_2"], axis=(ind_range)).astype(float)
scene_2_summed /= np.sum(scene_2_summed)
plt.plot(domain, scene_2_summed)
plt.ylim(0, 1.0)
plt.subplot(4, 2, 8)
scene_2_summed *= baseline_summed
scene_2_summed /= np.sum(scene_2_summed)
plt.plot(domain, scene_2_summed)
plt.ylim(0, 1.0)

plt.show()

