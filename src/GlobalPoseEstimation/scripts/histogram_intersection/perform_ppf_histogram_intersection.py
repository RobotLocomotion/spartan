import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import yaml

BASELINE_SUMMARY = "all_ppfs_summary.yaml"
TEMPLATE_HISTOGRAM = "drill_decimated_mesh_ppf_histogram.yaml"
SCENE_HISTOGRAM_1 = "scene_cloud_uncropped_ppf_histogram_object_neighborhood.yaml"
SCENE_HISTOGRAM_2 = "scene_cloud_uncropped_ppf_histogram_wrong_object_neighborhood.yaml"

histogram_filenames = {
    "template": TEMPLATE_HISTOGRAM,
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
    histograms[name] = np.reshape(histogram_data[name]["histogram"], dims, order="C").astype(float)

baseline_data = yaml.load(open(BASELINE_SUMMARY))
# Check dimensionality
this_dims = [
        baseline_data["n_bins"]["distance"],
        baseline_data["n_bins"]["n1_n2"],
        baseline_data["n_bins"]["d_n1"],
        baseline_data["n_bins"]["d_n2"]
    ]
this_maxdist = baseline_data["max_distance"]    
if this_dims != dims or this_maxdist != maxdist:
    print "Dims or maxdist don't match between input files!"
    exit(0)
baseline_counts = np.reshape(baseline_data["histogram_counts"], dims, order="C").astype(float)
baseline_means = np.reshape(baseline_data["histogram_means"], dims, order="C").astype(float)
baseline_variances = np.reshape(baseline_data["histogram_variances"], dims, order="C").astype(float)

maxondim = [maxdist, math.pi, math.pi, math.pi]
labels = ["distance", "n1_n2", "d_n1", "d_n2"]

plt.figure()

# Show all three base histograms
dim_ind = 3
ind_range = tuple(range(4)[0:dim_ind] + range(4)[dim_ind+1:])
domain = np.arange(dims[dim_ind])*maxondim[dim_ind]/dims[dim_ind]

# build baseline distribution on this dimension
# the mean is just a weighted average by sample size
this_dimension_means = np.sum(baseline_means * baseline_counts, axis=ind_range) / np.sum(baseline_counts, axis=ind_range)
# following a generalization of
# https://stats.stackexchange.com/questions/43159/how-to-calculate-pooled-variance-of-two-groups-given-known-group-variances-mean
this_dimension_variances = np.sum( (baseline_means + baseline_variances)*baseline_counts, axis=ind_range) / np.sum(baseline_counts, axis=ind_range)  - this_dimension_means

# Generate template histogram
template_summed = np.sum(histograms["template"], axis=(ind_range)).astype(float)
template_summed /= np.sum(template_summed)

# And scene histograms
scene_summed = np.sum(histograms["scene"], axis=(ind_range)).astype(float)
scene_summed /= np.sum(scene_summed)
scene_2_summed = np.sum(histograms["scene_2"], axis=(ind_range)).astype(float)
scene_2_summed /= np.sum(scene_2_summed)

def normpdf(x, mean, var, min_prob=0.0):
    pi = 3.1415926
    num = np.exp(-(x-mean)**2/(2.*var))
    return np.maximum(num, min_prob)
min_prob_in_use = 0.0

def plot_prior_and_template():
    # Plot the prior info
    plt.plot(domain, this_dimension_means, color="b")
    plt.fill(np.concatenate([domain, domain[::-1]]),
             np.concatenate([this_dimension_means - 1.96 * this_dimension_variances,
                            (this_dimension_means + 1.96 * this_dimension_variances)[::-1]]),
             alpha=0.5, fc='b', ec='None', label='95% confidence interval')

    # And the template, and its likelihood (pointwise) under the prior
    plt.plot(domain, template_summed, "r--")
    likelihood = np.log(normpdf(template_summed, this_dimension_means, this_dimension_variances, min_prob_in_use))
    template_scaled = template_summed * likelihood
    template_scaled /= template_scaled.sum()
    plt.plot(domain, template_scaled, color="r")

plt.subplot(2, 1, 1)
plot_prior_and_template()
# And a scene
plt.plot(domain, scene_summed, "g--")
likelihood = np.log(normpdf(scene_summed, this_dimension_means, this_dimension_variances, min_prob_in_use))
scene_scaled = scene_summed * likelihood
scene_scaled /= scene_scaled.sum()
plt.plot(domain, scene_scaled, color="g")

plt.subplot(2, 1, 2)
plot_prior_and_template()
# And a scene
plt.plot(domain, scene_2_summed, "g--")
likelihood = np.log(normpdf(scene_2_summed, this_dimension_means, this_dimension_variances, min_prob_in_use))
scene_2_scaled = scene_2_summed * likelihood
scene_2_scaled /= scene_2_scaled.sum()
plt.plot(domain, scene_2_scaled, color="g")

plt.show()
exit()


plt.figure()
k = 0
k+=1
plt.subplot(4, 3, k)
plt.ylabel("baseline")
plt.plot(domain, this_dimension_means)
plt.fill(np.concatenate([domain, domain[::-1]]),
         np.concatenate([this_dimension_means - 1.96 * this_dimension_variances,
                        (this_dimension_means + 1.96 * this_dimension_variances)[::-1]]),
         alpha=0.5, fc='b', ec='None', label='95% confidence interval')
k+=1
plt.subplot(4, 3, k)
plt.plot(domain, this_dimension_means)
plt.fill(np.concatenate([domain, domain[::-1]]),
         np.concatenate([this_dimension_means - 1.96 * this_dimension_variances,
                        (this_dimension_means + 1.96 * this_dimension_variances)[::-1]]),
         alpha=0.5, fc='b', ec='None', label='95% confidence interval')
k+=1
plt.subplot(4, 3, k)
plt.plot(domain, this_dimension_means)
plt.fill(np.concatenate([domain, domain[::-1]]),
         np.concatenate([this_dimension_means - 1.96 * this_dimension_variances,
                        (this_dimension_means + 1.96 * this_dimension_variances)[::-1]]),
         alpha=0.5, fc='b', ec='None', label='95% confidence interval')


k+=1
plt.subplot(4, 3, k)
plt.ylabel("template")
plt.plot(domain, template_summed)
plt.ylim(0, 1.0)
k+=1
plt.subplot(4, 3, k)
template_summed = template_summed - this_dimension_means
plt.plot(domain, template_summed)
plt.ylim(-1., 1.0)
k+=1
plt.subplot(4, 3, k)


k+=1
plt.subplot(4, 3, k)
plt.ylabel("correct object scene")
plt.plot(domain, scene_summed)
plt.ylim(0, 1.0)
k+=1
plt.subplot(4, 3, k)
scene_summed = scene_summed - this_dimension_means
plt.plot(domain, scene_summed)
plt.ylim(-1.0, 1.0)
k+=1
plt.subplot(4, 3, k)


k+=1
plt.subplot(4, 3, k)
plt.ylabel("wrong object scene")
plt.plot(domain, scene_2_summed)
plt.ylim(0, 1.0)
k+=1
plt.subplot(4, 3, k)
scene_2_summed = scene_2_summed - this_dimension_means
plt.plot(domain, scene_2_summed)
plt.ylim(-1.0, 1.0)
k+=1
plt.subplot(4, 3, k)


def compute_intersection_score(hist1, hist2):
    return np.sum([abs(x-y) for x, y in zip(hist1, hist2)])
score_correct = compute_intersection_score(scene_summed, template_summed)
score_incorrect = compute_intersection_score(scene_2_summed, template_summed)
print "Scores:"
print "\tCorrect:", score_correct
print "\tIncorrect:", score_incorrect


plt.show()
