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

dat = np.reshape(histogram_info["histogram"], dims, order="C")

plt.figure()

plt.title("Histogram in each dimension")

for i in range(4):
	plt.subplot(2, 2, i+1)
	axis = range(4)[0:i] + range(4)[i+1:]
	plt.plot(np.arange(dims[i])*maxondim[i]/dims[i], np.sum(dat, axis=tuple(axis)))


plt.figure()

xedges = np.linspace(0, maxdist, dims[0])
yedges = np.linspace(0, math.pi, dims[1])
H = np.sum(dat, axis=(2, 3))
from sklearn.preprocessing import normalize
H = normalize(H, axis=1, norm="l1")
X, Y = np.meshgrid(xedges, yedges)
plt.pcolormesh(X, Y, H)
plt.colorbar()
plt.ylabel("Angle n1 n2")
plt.xlabel("Dist")
plt.title("Dist vs N1 N2 Angular Distance, Normalized Across Distance")


plt.show()

