#!/usr/bin/env python

import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import signal
from scipy import stats

data_files = [
	"out_lin_2bin_15s_5o_0n.yaml"
]

all_corruptions = []
all_solve_times = []
all_objectives = []

for file in data_files:
	data = yaml.load(open(file))
	#all_corruptions.extend(data["corruption_amounts"])
	all_solve_times.extend(data["solve_times"])
	all_objectives.extend(data["objectives"])

N = 20
#all_corruptions = np.array(all_corruptions)
all_solve_times = np.array(all_solve_times)
bin_means, bin_edges, binnumber = stats.binned_statistic(all_corruptions, all_solve_times, statistic="mean", bins=N)
#bin_stds = stats.binned_statistic(all_corruptions, all_solve_times, statistic="std", bins=N)

print bin_means, bin_edges, bin_stds

plt.figure()
plt.scatter(all_corruptions, all_solve_times, color="r", label="samples")
plt.hlines(bin_means, bin_edges[:-1], bin_edges[1:], color='g', lw=10, label="means")
plt.legend(fontsize=10)
plt.show()
'''
unique_corruptions = np.unique(all_corruptions)
solve_time_avg = []
solve_time_var = []
n = []
for amt in unique_corruptions:
	these_solve_times = []
	for i, x in enumerate(all_corruptions):
		if amt == x and all_objectives[i] == 0.0:
			these_solve_times.append(all_solve_times[i])
	solve_time_avg.append( np.mean(these_solve_times ) )
	solve_time_var.append( np.sqrt(np.var(these_solve_times )) )
	n.append( len(these_solve_times) )
	print these_solve_times

print unique_corruptions
print solve_time_avg
print solve_time_var
print n
plt.errorbar(unique_corruptions, solve_time_avg, solve_time_var)
plt.xlabel("corruption amt (m or radians)")
plt.ylabel("solve time")
plt.show()
'''
