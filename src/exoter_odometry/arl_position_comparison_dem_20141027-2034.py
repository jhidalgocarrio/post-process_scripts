#!/usr/bin/env python

path = '~/npi/data/20141024_planetary_lab/20141027-2034_odometry_comparison_review/'

#######################################
threed_odometry_file = path + 'pose_odo_position.no_forces.0.data'

threed_odometry_with_forces_file = path + 'pose_odo_position.reaction_forces.0.data'

skid_odometry_file = path + 'pose_skid_position.0.data'

contact_odometry_file = path + 'pose_contact_odometry_position.0.data'

reference_file = path + 'pose_ref_position.0.data'

navigation_orientation_file = path + 'pose_world_to_navigation_orientation.0.data'

navigation_position_file = path + 'pose_world_to_navigation_position.0.data'
#######################################
esa_arl_dem_file = '~/npi/documentation/esa_terrain_lab/DEMclean.ply'
#######################################

import sys
sys.path.insert(0, './src/core')
import os
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

import datetime
matplotlib.style.use('classic') #in matplotlib >= 1.5.1

#ExoTeR Odometry
threed_odometry = data.ThreeData()
threed_odometry.readData(threed_odometry_file, cov=True)

#ExoTeR Odometry
threed_odometry_forces = data.ThreeData()
threed_odometry_forces.readData(threed_odometry_with_forces_file, cov=True)

#Skid Odometry
skid_odometry = data.ThreeData()
skid_odometry.readData(skid_odometry_file, cov=True)

#Contact Odometry
contact_odometry = data.ThreeData()
contact_odometry.readData(contact_odometry_file, cov=True)

#Vicon Pose
reference = data.ThreeData()
reference.readData(reference_file, cov=True)

#World to Navigation Pose
navigation_orient = data.QuaternionData()
navigation_orient.readData(navigation_orientation_file, cov=False)
navigation_position = data.ThreeData()
navigation_position.readData(navigation_position_file, cov=False)

########################
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference.cov[:,0,0]))
temindex = np.asarray(temindex)

reference.delete(temindex)
threed_odometry.delete(temindex)
threed_odometry_forces.delete(temindex)
skid_odometry.delete(temindex)
contact_odometry.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
reference.eigenValues()
threed_odometry.eigenValues()
threed_odometry_forces.eigenValues()
contact_odometry.eigenValues()


############
### PLOT ###
############

# Terrain DEM
plydata = PlyData.read(open(os.path.expanduser(esa_arl_dem_file)))

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
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)
#fig, ax = plt.subplots()

# Display the DEM
plt.rc('text', usetex=False)# activate latex text rendering
CS = plt.contour(xi, yi, zi, 15, linewidths=0.5, colors='k')
CS = plt.contourf(xi, yi, zi, 15, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())

# plot data points.
plt.xlim(min(px), max(xi))
plt.ylim(min(py), max(yi))


# Color bar of the dem
cbar = plt.colorbar()  # draw colorbar
cbar.ax.set_ylabel(r' terrain elevation [$m$]', fontsize=25, fontweight='bold')

# Odometry trajectory with reaction_forces
plt.rc('text', usetex=False)# activate latex text rendering
xposition = threed_odometry_forces.getAxis(0)[0::50]
yposition = threed_odometry_forces.getAxis(1)[0::50]
zposition = threed_odometry_forces.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Odometry trajectory
x = position[:,0]
y = position[:,1]
ax.plot(x, y, marker='o', linestyle='-.', label="E. 3D odometry", color=[0.7,1.0,0.4], lw=5)

# Odometry trajectory
plt.rc('text', usetex=False)# activate latex text rendering
xposition = threed_odometry.getAxis(0)[0::50]
yposition = threed_odometry.getAxis(1)[0::50]
zposition = threed_odometry.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Odometry trajectory
x = position[:,0]
y = position[:,1]
ax.plot(x, y, marker='o', linestyle='-.', label="3D odometry", color=[0.3,1.0,0.4], lw=5)


# Planar Odometry trajectory
plt.rc('text', usetex=False)# activate latex text rendering
xposition = skid_odometry.getAxis(0)[0::50]
yposition = skid_odometry.getAxis(1)[0::50]
zposition = skid_odometry.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Planar Odometry trajectory
x = position[:,0]
y = position[:,1]
ax.plot(x, y, marker='x', linestyle='--', label="skid odometry", color=[0,0.5,1], lw=5)

# Contact Odometry trajectory
plt.rc('text', usetex=False)# activate latex text rendering
xposition = contact_odometry.getAxis(0)[0::50]
yposition = contact_odometry.getAxis(1)[0::50]
zposition = contact_odometry.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Planar Odometry trajectory
#x = position[:,0]
#y = position[:,1]
#ax.plot(x, y, marker='^', linestyle='-', label="Contact Point Odometry", color=[0.3,0.5,1], lw=5)

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
ax.plot(x, y, marker='D', linestyle='--', label="reference trajectory", color=[0.5,0,0], alpha=0.5, lw=5)

from matplotlib.cbook import get_sample_data
from matplotlib._png import read_png
import matplotlib.image as image
from scipy import ndimage
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
fn = get_sample_data(os.getcwd()+"/data/img/exoter.png", asfileobj=False)
exoter = image.imread(fn)
exoter = ndimage.rotate(exoter, 180)
imexoter = OffsetImage(exoter, zoom=0.5)


ab = AnnotationBbox(imexoter, xy=(x[0], y[0]),
            xybox=None,
            xycoords='data',
            boxcoords="offset points",
            frameon=False)

ax.annotate(r'ExoTeR', xy=(x[0], y[0]), xycoords='data',
                    xytext=(-40, 50), textcoords='offset points', fontsize=30,
                    #arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=5.0)
                    zorder=101
                    )

ax.annotate(r'Start', xy=(x[0], y[0]), xycoords='data',
                    xytext=(-5, 5), textcoords='offset points', fontsize=30,
                    horizontalalignment='left',
                    verticalalignment='bottom',
                    zorder=101
                    )
ax.scatter(x[0], y[0], marker='o', facecolor='k', s=100, alpha=1.0, zorder=103)

ax.arrow(x[0], y[0], x[32]-x[0], y[32]-y[0], width=0.02, head_width=0.07,
    head_length=0.1, fc='k', ec='k', zorder=104)

# End sign
ax.annotate(r'End', xy=(x[x.shape[0]-1], y[y.shape[0]-1]), xycoords='data',
                    xytext=(-5, 5), textcoords='offset points', fontsize=30,
                    horizontalalignment='left',
                    verticalalignment='bottom',
                    zorder=101
                    )
ax.scatter(x[x.shape[0]-1], y[y.shape[0]-1], marker='o', facecolor='k', s=100, alpha=1.0, zorder=103)

ax.add_artist(ab)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
ax.legend(loc=4, prop={'size':30})
#plt.axis('equal')
fig.savefig("arl_odometry_comparison_20141027-2034.pdf", dpi=fig.dpi)
plt.grid(True)
plt.show(block=True)



##################3
# 3D Ploting
#
#

from mpl_toolkits.mplot3d import axes3d

fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
ax = fig.gca(projection='3d')
x, y = np.meshgrid(xi, yi)
ax.plot_surface(x, y, zi, rstride=2, cstride=2, alpha=1.0, linewidth=0.3, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())
cset = ax.contour(x, y, zi, zdir='z', cmap=plt.cm.coolwarm,
        vmax=abs(zi).max(), vmin=-abs(zi).max(), linewidth=4.0)



# Odometry trajectory
xposition = threed_odometry.getAxis(0)[0::50]
yposition = threed_odometry.getAxis(1)[0::50]
zposition = threed_odometry.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Odometry trajectory
x = position[:,0]
y = position[:,1]
z = position[:,2]
ax.plot(x, y, z, marker='o', linestyle='-.', label="Enhanced 3D Odometry", color=[0.3,1.0,0.4], lw=5)

# Planar Odometry trajectory
plt.rc('text', usetex=False)# activate latex text rendering
xposition = skid_odometry.getAxis(0)[0::50]
yposition = skid_odometry.getAxis(1)[0::50]
zposition = skid_odometry.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Planar Odometry trajectory
x = position[:,0]
y = position[:,1]
z = position[:,2]
ax.plot(x, y, z, marker='x', linestyle='--', label="Planar Odometry", color=[0,0.5,1], lw=5)

# Contact Odometry trajectory
plt.rc('text', usetex=False)# activate latex text rendering
xposition = contact_odometry.getAxis(0)[0::50]
yposition = contact_odometry.getAxis(1)[0::50]
zposition = contact_odometry.getAxis(2)[0::50]

# rotate and translate the trajectory wrt the world frame
position = np.column_stack((xposition, yposition, zposition))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Display Planar Odometry trajectory
x = position[:,0]
y = position[:,1]
z = position[:,2]
ax.plot(x, y, z, marker='^', linestyle='.-', label="Contact Point Odometry", color=[0.3,0.5,1], lw=5)

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
z = position[:,2]
ax.plot(x, y, z, marker='D', linestyle='--', label="Reference Trajectory", color=[0.5,0,0], alpha=0.5, lw=5)

ax.set_xlabel('X')
ax.set_xlim(0, max(xi))
ax.set_ylabel('Y')
ax.set_ylim(0, max(yi))
ax.set_zlabel('Z')
ax.set_zlim(0, 4)

ax.legend(loc=2, prop={'size':30})
plt.show(block=False)

