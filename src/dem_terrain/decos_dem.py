#!/usr/bin/env python

#######################################
#decos_dem_file = '/home/javi/exoter/development/decos_terrain/2014_10_17_test_geotag_filtered_segmented_point_cloud.ply'
decos_dem_file = '/home/javi/exoter/development/decos_terrain/decos_small_testing_area_point_cloud.ply'
#######################################

import sys
sys.path.insert(0, './src/core')
from numpy.random import uniform, seed
from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib import cm
import numpy as np
from plyfile import PlyData, PlyElement
import scipy

plydata = PlyData.read(open(decos_dem_file))

vertex = plydata['vertex'].data

[px, py, pz] = (vertex[t] for t in ('x', 'y', 'z'))

# define grid.
npts=500
xi = np.linspace(min(px), max(px), npts)
yi = np.linspace(min(py), max(py), npts)

# grid the data.
zi = griddata(px, py, pz, xi, yi, interp='linear')

############
### PLOT ###
############
fig = plt.figure(1)

plt.rc('text', usetex=False)# activate latex text rendering
levs = np.arange(np.floor(zi.min()), np.ceil(zi.max()), 0.1)
CS = plt.contour(xi, yi, zi, levs, linewidths=0.5, colors='k')
CS = plt.contourf(xi, yi, zi, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())
plt.colorbar()  # draw colorbar
# plot data points.
plt.xlim(min(px), max(xi))
plt.ylim(min(py), max(yi))
#plt.title('griddata test (%d points)' % npts)
plt.show(block=False)

############
# 3D plotinh
x, y = np.meshgrid(xi, yi)

############
fig = plt.figure()
ax = fig.gca(projection='3d')
surf = ax.plot_surface(x, y, zi, rstride=15, cstride=15, cmap=cm.coolwarm, linewidth=1, antialiased=True)
fig.colorbar(surf)

plt.show()

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(x, y, zi, rstride=2, cstride=2, alpha=1.0, linewidth=0.1, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())
cset = ax.contour(x, y, zi, zdir='z', cmap=plt.cm.coolwarm,
        vmax=abs(zi).max(), vmin=-abs(zi).max(), linewidth=4.0)
cset = ax.contour(x, y, zi, zdir='x', cmap=plt.cm.gray, vmax=abs(xi).max(), vmin=-abs(xi).max())
cset = ax.contour(x, y, zi, zdir='y', cmap=plt.cm.gray, vmax=abs(yi).max(), vmin=-abs(yi).max())

ax.set_xlabel('X')
ax.set_xlim(0, max(xi))
ax.set_ylabel('Y')
ax.set_ylim(0, max(yi))
ax.set_zlabel('Z')
ax.set_zlim(0, 10)

plt.show(block=False)




