#!/usr/bin/env python

import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import signal
from scipy import stats

input_solves_files = ["two_cubes_example_run_with_icp.yaml",
		"two_cubes_example_run_without_icp.yaml"]
colors = ["g", "r"]
names = ["With ICP", 
		  "Without ICP"]

import collections
DataContainer = collections.namedtuple("DataContainer", ["elapsed_times", "objectives", "bounds", "color", "name"])

containers = []
for i, file in enumerate(input_solves_files):
	data = yaml.load(open(file))
	containers.append( DataContainer(
	 	np.array(data[0]["history"]["reported_runtime"]),
		np.array(data[0]["history"]["best_objective"]),
		np.array(data[0]["history"]["best_bound"]),
		colors[i],
		names[i]))


plt.figure()

max_time = 0
max_bound = 0
for container in containers:
	plt.plot(container.elapsed_times, container.objectives, '-', linewidth=5.0, color=container.color, label=container.name + " upper bound")
	plt.plot(container.elapsed_times, container.bounds, '--', linewidth=5.0, color=container.color, label=container.name + " lower bound")
	max_time = max(max_time, np.max(container.elapsed_times))
	max_bound = max(max_bound, np.max(container.objectives[10:]))

plt.ylabel("Average L1 Error over Scene Points (m)")
plt.grid()
plt.axhline(0.025, color='b', linestyle='dashed', linewidth=2, label="Optimal Solution")
plt.legend(fontsize=10)
plt.xlabel("Elapsed solve time (s)")
plt.ylim(-0.01, 0.08)
plt.xlim(0, max_time+1)
plt.show()