#!/usr/bin/env python

path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141025-0005/'
#######################################
path_odometry_file = path + 'pose_odo_position.0.data'

path_reference_file = path + 'pose_ref_position.0.data'

path_navigation_orientation_file = path + 'pose_world_to_navigation_orientation.0.data'

path_navigation_position_file = path + 'pose_world_to_navigation_position.0.data'
#######################################
esa_arl_dem_file = '/home/javi/exoter/development/esa_terrain_lab/DEMclean.ply'
#######################################


import sys
sys.path.insert(0, './src/core')
import csv
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov
from plyfile import PlyData, PlyElement
import scipy


#ExoTeR Odometry
odometry = data.ThreeData()
odometry.readData(path_odometry_file, cov=True)

#Vicon Pose
reference = data.ThreeData()
reference.readData(path_reference_file, cov=True)

#World to Navigation Pose
navigation_orient = data.QuaternionData()
navigation_orient.readData(path_navigation_orientation_file, cov=False)
navigation_position = data.ThreeData()
navigation_position.readData(path_navigation_position_file, cov=False)

########################
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference.data[:,0]))
temindex = np.asarray(temindex)

odometry.delete(temindex)
reference.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
odometry.covSymmetry()
odometry.eigenValues()
reference.covSymmetry()
reference.eigenValues()

# Terrain DEM
plydata = PlyData.read(open(esa_arl_dem_file))

vertex = plydata['vertex'].data

[px, py, pz] = (vertex[t] for t in ('x', 'y', 'z'))

# define grid.
npts=100
xi = np.linspace(min(px), max(px), npts)
yi = np.linspace(min(py), max(py), npts)

# grid the data.
zi = griddata(px, py, pz, xi, yi)

############
### PLOT ###
############
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

# Display the DEM
plt.rc('text', usetex=False)# activate latex text rendering
CS = plt.contour(xi, yi, zi, 15, linewidths=0.5, colors='k')
CS = plt.contourf(xi, yi, zi, 15, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())
cbar = plt.colorbar()  # draw colorbar
cbar.ax.set_ylabel('terrain elevation')
# plot data points.
plt.xlim(min(px), max(xi))
plt.ylim(min(py), max(yi))

# Display Odometry trajectory


# rotate and translate the trajectory wrt the world frame
position = np.column_stack((reference.getAxis(0)[0::50], reference.getAxis(1)[0::50],  reference.getAxis(2)[0::50]))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Ground Truth trajectory
x = position[:,0]
y = position[:,1]

Q = ax.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy',
        angles='xy', scale=1, color='r', units='x', linewidths=(1,),
        edgecolors=('k'), headaxislength=5)
qk = ax.quiverkey(Q, 0.9, 0.02, 0.5,  'trajectory line', fontproperties={'weight': 'bold', 'size':16})

import os
from matplotlib.cbook import get_sample_data
from matplotlib._png import read_png
import matplotlib.image as image
from scipy import ndimage
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
fn = get_sample_data(os.getcwd()+"/data/img/exoter.png", asfileobj=False)
exoter = image.imread(fn)
exoter = ndimage.rotate(exoter, 180)
imexoter = OffsetImage(exoter, zoom=0.3)


ab = AnnotationBbox(imexoter, xy=(x[0], y[0]),
                        xybox=None,
                        xycoords='data',
                        boxcoords="offset points",
                        frameon=False)

ax.annotate("ExoTeR", xy=(x[0], y[0]), xycoords='data',
                                xytext=(-40, 45), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

ax.add_artist(ab)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(handles=[exoter], loc=1, prop={'size':30})
plt.show(block=False)


