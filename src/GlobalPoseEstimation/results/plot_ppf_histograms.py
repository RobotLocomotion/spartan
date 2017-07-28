import numpy as np
import matplotlib.pyplot as plt
import sys

dat = np.loadtxt(open(sys.argv[1]), delimiter=",")
# cleanup bad values
print "Removing %d bad values with avg dist %f" % (
        np.sum(np.any(np.invert(np.isfinite(dat)), axis=1)),
        np.mean( dat[np.any(np.invert(np.isfinite(dat)), axis=1), 2] )
    )
dat = dat[np.all(np.isfinite(dat), axis=1)]

dat = dat[dat[:, 2] < 0.2, :]

i_1 = dat[:, 0]
i_2 = dat[:, 1]
d = dat[:, 2]
n1n2 = dat[:, 3]
dn1 = dat[:, 4]
dn2 = dat[:, 5]


plt.figure()

# Independently
plt.subplot(2, 2, 1)
plt.hist(d, bins='auto')
plt.ylabel("Dist")

plt.subplot(2, 2, 2)
plt.hist(n1n2, bins='auto')
plt.ylabel("Angle n1 n2")

plt.subplot(2, 2, 3)
plt.hist(dn1, bins='auto')
plt.ylabel("Angle d n1")

plt.subplot(2, 2, 4)
plt.hist(dn2, bins='auto')
plt.ylabel("Angle d n2")

plt.figure()
H, xedges, yedges = np.histogram2d(d, n1n2, bins=15)
from sklearn.preprocessing import normalize
H = normalize(H, axis=1, norm="l1")
X, Y = np.meshgrid(xedges, yedges)
plt.pcolormesh(X, Y, H)
plt.colorbar()
plt.ylabel("Angle n1 n2")
plt.xlabel("Dist")
plt.title("Dist vs N1 N2 Angular Distance, Normalized Across Distance")


plt.show()

