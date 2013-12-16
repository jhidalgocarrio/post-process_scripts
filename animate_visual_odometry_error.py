import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from matplotlib import patches
import pointcloud_display as point
import datadisplay as data
import error_ellipse as ellip


vodoFeatures = point.PointCloud()
vodoFeatures.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131210-1926/data/pointcloud_position.0.data',
                      '/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131210-1926/data/pointcloud_cov.0.data', cov=True)
vodoFeatures.eigenValues()

fig = plt.figure(1)#figsize=(8,6), dpi=300)
ax = fig.add_subplot(111)
ax.set_xlim(-0.6, 0.5)
ax.set_ylim(0.0, 2.0)
ax.grid(True)
#xdata, ydata = [], []

for i in range (2):
    x = vodoFeatures.data[i][0]
    z = vodoFeatures.data[i][2]
    pos = np.array([x,z])
    cov = np.array([[vodoFeatures.cov[i][0][0], vodoFeatures.cov[i][0][2]],
                    [vodoFeatures.cov[i][2][0], vodoFeatures.cov[i][2][2]]])

    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))+30
    nstd=15

    # Width and height are "full" widths, not radius
    width, height = 2 * nstd * np.sqrt(vals)
    ellip = patches.Ellipse(xy=pos, width=width, height=height, angle=theta, alpha=0.5, color='green')

    plt.plot(x,z, linestyle='none', marker='o', color='red')
    ax.add_artist(ellip)
    #plt.draw()
    #plt.savefig('movie/visual_odometry_uncertainty_'+str(i)+'.png')


plt.show(block=False)

# Figure
fig = plt.figure(2)
ax = fig.add_subplot(111)
ax.set_xlim(-0.6, 0.5)
ax.set_ylim(0.0, 2.0)
ax.grid(False)

# One Ellipse
i=2
x = vodoFeatures.data[i][0]
z = vodoFeatures.data[i][2]
pos = np.array([x,z])
cov = np.array([[vodoFeatures.cov[i][0][0], vodoFeatures.cov[i][0][2]],
                [vodoFeatures.cov[i][2][0], vodoFeatures.cov[i][2][2]]])

def eigsorted(cov):
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    return vals[order], vecs[:,order]

vals, vecs = eigsorted(cov)
theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))+30
nstd=15

# Width and height are "full" widths, not radius
width, height = 2 * nstd * np.sqrt(vals)
ellip = patches.Ellipse(xy=pos, width=width, height=height, angle=theta, alpha=0.5, color='green')
e1 = patches.Arc((x,z), width, height, angle=theta, linewidth=2, fill=False, zorder=2)

plt.plot(x,z, linestyle='none', marker='o', color='red')
ax.add_artist(ellip)
ax.add_patch(e1)
plt.show(block=False)

plt.savefig('figures/feature_uncertainty.png')


