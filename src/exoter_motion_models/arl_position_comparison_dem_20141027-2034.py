#!/usr/bin/env python

path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034_odometry_comparison/'

#######################################
path_odometry_file = path + 'pose_odo_position.reaction_forces.0.data'

path_skid_file = path + 'pose_skid_position.0.data'

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

#Skid Odometry
skid = data.ThreeData()
skid.readData(path_skid_file, cov=True)

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
temindex = np.where(np.isnan(reference.cov[:,0,0]))
temindex = np.asarray(temindex)

reference.delete(temindex)
odometry.delete(temindex)
skid.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
reference.eigenValues()
odometry.eigenValues()
skid.eigenValues()

############
### PLOT ###
############

# Terrain DEM
plydata = PlyData.read(open(esa_arl_dem_file))

vertex = plydata['vertex'].data

[px, py, pz] = (vertex[t] for t in ('x', 'y', 'z'))

# define grid.
npts=100
xi = np.linspace(min(px), max(px), npts)
yi = np.linspace(min(py), max(py), npts)

# grid the data.
zi = griddata(px, py, pz, xi, yi, interp='linear')

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

# Odometry trajectory
plt.rc('text', usetex=False)# activate latex text rendering
xposition = odometry.getAxis(0)[0::50]
yposition = odometry.getAxis(1)[0::50]
zposition = odometry.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Odometry trajectory
x = position[:,0]
y = position[:,1]
ax.plot(x, y, marker='o', linestyle='-.', label="Enhanced 3D Odometry", color=[0.3,1.0,0.4], lw=2)


# Planar Odometry trajectory
plt.rc('text', usetex=False)# activate latex text rendering
xposition = skid.getAxis(0)[0::50]
yposition = skid.getAxis(1)[0::50]
zposition = skid.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Planar Odometry trajectory
x = position[:,0]
y = position[:,1]
ax.plot(x, y, marker='x', linestyle='--', label="Planar Odometry", color=[0,0.5,1], lw=2)


# Reference trajectory
plt.rc('text', usetex=False)# activate latex text rendering
xposition = reference.getAxis(0)[0::50]
yposition = reference.getAxis(1)[0::50]
zposition = reference.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Reference trajectory
x = position[:,0]
y = position[:,1]
ax.plot(x, y, marker='D', linestyle='--', label="Reference Trajectory", color=[0.5,0,0], alpha=0.5, lw=2)

# Start and End Labels
ax.scatter(x[0], y[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
ax.scatter(x[len(x)-1], y[len(y)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
ax.annotate(r'Start', xy=(x[0], y[0]), xycoords='data',
                                xytext=(-40, -40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))
ax.annotate(r'End', xy=(x[len(xposition)-1], y[len(yposition)-1]), xycoords='data',
                                xytext=(-40, +40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))


plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
ax.legend(loc=2, prop={'size':30})
plt.axis('equal')
plt.grid(True)
plt.show(block=False)
#plt.show(block=True)


