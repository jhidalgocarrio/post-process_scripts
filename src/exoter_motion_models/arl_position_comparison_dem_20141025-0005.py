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
plt.colorbar()  # draw colorbar
# plot data points.
plt.xlim(min(px), max(xi))
plt.ylim(min(py), max(yi))

# Display Odometry trajectory


# rotate and translate the trajectory wrt the world frame
position = np.column_stack((reference.getAxis(0)[0::50], reference.getAxis(1)[0::50],  reference.getAxis(2)[0::50]))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Ground Truth trajectory
xposition, yposition = np.meshgrid(position[:,0], position[:, 1])
xposition = position[:,0]
yposition = position[:,1]

ax.quiver (X=xposition, Y=yposition, U=5*xposition, V=5*yposition)
ax.plot(xposition, yposition, marker='<', linestyle='None', label="Rover Trajectory", color=[0.5,0,0], alpha=0.5, lw=2)

plt.show(block=False)


n = 8
X,Y = meshgrid(position[0:n,0],position[0:n, 1])
T = np.arctan2(Y-n/2.0, X-n/2.0)
R = 10+np.sqrt((Y-n/2.0)**2+(X-n/2.0)**2)
U,V = R*np.cos(T), R*np.sin(T)

axes([0.025,0.025,0.95,0.95])
quiver(X,Y,U,V,R, alpha=.5)
quiver(X,Y,U,V, edgecolor='k', facecolor='None', linewidth=.5)
