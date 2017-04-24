#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2017-03-27 18:36:34

path = '~/npi/data/20150515_planetary_lab/20150515-1752_odometry_comparison/'

#######################################
threed_odometry_file = path + 'pose_odo_position.0.data'

skid_odometry_file = path + 'pose_skid_position.0.data'

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
from numpy import linalg as la

import pandas as pandas
import datetime
matplotlib.style.use('ggplot') #in matplotlib >= 1.5.1

def dateparse (time_in_microsecs):
    return datetime.datetime.fromtimestamp(float(time_in_microsecs) * 1e-06)

##########################################################################
# PLOTTING FUNCTION
##########################################################################
def arl_odometry_dem_figure(fig_num, dem_file, threed_odometry_trajectory, skid_odometry_trajectory, reference_trajectory):

    ########################
    # Load Terrain DEM
    ########################
    import os
    plydata = PlyData.read(open(os.path.expanduser(dem_file)))

    vertex = plydata['vertex'].data

    [dem_px, dem_py, dem_pz] = (vertex[t] for t in ('x', 'y', 'z'))

    # define grid.
    npts=100
    dem_xi = np.linspace(min(dem_px), max(dem_px), npts)
    dem_yi = np.linspace(min(dem_py), max(dem_py), npts)

    # grid the data.
    dem_zi = griddata(dem_px, dem_py, dem_pz, dem_xi, dem_yi, interp='linear')


    matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
    fig = plt.figure(fig_num, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.add_subplot(111)
    #fig, ax = plt.subplots()

    # Display the DEM
    plt.rc('text', usetex=False)# activate latex text rendering
    CS = plt.contour(dem_xi, dem_yi, dem_zi, 15, linewidths=0.5, colors='k')
    CS = plt.contourf(dem_xi, dem_yi, dem_zi, 15, cmap=plt.cm.gray, vmax=abs(dem_zi).max(), vmin=-abs(dem_zi).max())

    # plot data points.
    plt.xlim(min(dem_px), max(dem_xi))
    plt.ylim(min(dem_py), max(dem_yi))

    # Color bar of the dem
    cbar = plt.colorbar()  # draw colorbar
    cbar.ax.set_ylabel(r' terrain elevation [$m$]', fontsize=25, fontweight='bold')

    # Plot 3d odometry trajectory
    x = threed_odometry_trajectory[:,0][0::5]
    y = threed_odometry_trajectory[:,1][0::5]
    ax.plot(x, y, marker='o', linestyle='-.', label="3d odometry", color=[0.3,1.0,0.4], lw=2)

    # Plot skid odometry
    x = skid_odometry_trajectory[:,0][0::5]
    y = skid_odometry_trajectory[:,1][0::5]
    ax.plot(x, y, marker='x', linestyle='--', label="skid odometry", color=[0,0.5,1], lw=2)

    x = reference_trajectory[1:reference_trajectory.shape[0]-1,0][0::5]
    y = reference_trajectory[1:reference_trajectory.shape[0]-1,1][0::5]
    ax.plot(x, y, marker='D', linestyle='--', label="reference trajectory", color=[0.5,0,0], alpha=0.5, lw=2)

    import os
    from matplotlib.cbook import get_sample_data
    from matplotlib._png import read_png
    import matplotlib.image as image
    from scipy import ndimage
    from matplotlib.offsetbox import OffsetImage, AnnotationBbox
    fn = get_sample_data(os.getcwd()+"/data/img/exoter.png", asfileobj=False)
    exoter = image.imread(fn)
    exoter = ndimage.rotate(exoter, 100)
    imexoter = OffsetImage(exoter, zoom=0.5)


    ab = AnnotationBbox(imexoter, xy=(x[0], y[0]),
                    xybox=None,
                    xycoords='data',
                    boxcoords="offset points",
                    frameon=False)

    ax.annotate(r'ExoTeR', xy=(x[1], y[1]), xycoords='data',
                            xytext=(-20, 30), textcoords='offset points', fontsize=12, zorder=101,
                            #arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0)
                            )

    ax.annotate(r'Start', xy=(x[0], y[0]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points', fontsize=12,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[0], y[0], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)

    ax.arrow(x[0], y[0], x[13]-x[0], y[13]-y[0], width=0.02, head_width=0.1,
            head_length=0.05, fc='k', ec='k', zorder=104)

    # End sign
    ax.annotate(r'End', xy=(x[x.shape[0]-1], y[y.shape[0]-1]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points', fontsize=12,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[x.shape[0]-1], y[y.shape[0]-1], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)

    ax.add_artist(ab)

    plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
    plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
    ax.legend(loc=2, prop={'size':30})
    #plt.axis('equal')
    plt.grid(True)
    fig.savefig("arl_odometry_comparison_20150515-1752.png", dpi=fig.dpi)
    plt.show(block=True)


##########################################################################
# READ THE VALUES IN PANDAS
##########################################################################

#ExoTeR Odometry
threed_odometry = pandas.read_csv(os.path.expanduser(threed_odometry_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#Skid Odometry
skid_odometry = pandas.read_csv(os.path.expanduser(skid_odometry_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#Vicon Pose
reference = pandas.read_csv(os.path.expanduser(reference_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)


#World to Navigation Pose
navigation_orient = data.QuaternionData()
navigation_orient.readData(navigation_orientation_file, cov=False)
navigation_position = data.ThreeData()
navigation_position.readData(navigation_position_file, cov=False)

#################
## RE-SAMPLE   ##
#################
resampling_time = '1s'
threed_odometry = threed_odometry.resample(resampling_time).mean()
skid_odometry = skid_odometry.resample(resampling_time).mean()
reference = reference.resample(resampling_time).mean()

##########################################################################
#rotate and translate the 3d odometry trajectory wrt the world frame
##########################################################################
threed_odometry_position = np.column_stack((threed_odometry.x.values, threed_odometry.y.values,  threed_odometry.z.values ))
threed_odometry_position[:] = [navigation_orient.data[0].rot(x) + navigation_position.data[0] for x in threed_odometry_position]

##########################################################################
#rotate and translate the skid odometry trajectory wrt the world frame
##########################################################################
skid_odometry_position = np.column_stack((skid_odometry.x.values, skid_odometry.y.values,  skid_odometry.z.values ))
skid_odometry_position[:] = [navigation_orient.data[0].rot(x) + navigation_position.data[0] for x in skid_odometry_position]

##########################################################################
#rotate and translate the reference trajectory wrt the world frame
##########################################################################
reference_position = np.column_stack((reference.x.values, reference.y.values,  reference.z.values ))
reference_position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in reference_position]

############
### PLOT ###
############
arl_odometry_dem_figure(1, esa_arl_dem_file, threed_odometry_position, skid_odometry_position, reference_position)

##################3
# 3D Ploting
#
#

from mpl_toolkits.mplot3d import axes3d

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

fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
ax = fig.gca(projection='3d')
x, y = np.meshgrid(xi, yi)
ax.plot_surface(x, y, zi, rstride=2, cstride=2, alpha=1.0, linewidth=0.3, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())
cset = ax.contour(x, y, zi, zdir='z', cmap=plt.cm.coolwarm,
        vmax=abs(zi).max(), vmin=-abs(zi).max(), linewidth=4.0)


plt.rc('text', usetex=False)# activate latex text rendering

# Odometry trajectory
xposition = threed_odometry_position[:,0][0::5]
yposition = threed_odometry_position[:,1][0::5]
zposition = threed_odometry_position[:,2][0::5]

# Display Odometry trajectory
x = xposition
y = yposition
z = zposition
ax.plot(x, y, z, marker='o', linestyle='-.', label="3d odometry", color=[0.3,1.0,0.4], lw=2)

# Planar Odometry trajectory
xposition = skid_odometry_position[:,0][0::5]
yposition = skid_odometry_position[:,1][0::5]
zposition = skid_odometry_position[:,2][0::5]

# Display Planar Odometry trajectory
x = xposition
y = yposition
z = zposition
ax.plot(x, y, z, marker='x', linestyle='--', label="skid odometry", color=[0,0.5,1], lw=2)

# Reference trajectory
xposition = reference_position[:,0][0::5]
yposition = reference_position[:,1][0::5]
zposition = reference_position[:,2][0::5]

# Display Reference trajectory
x = xposition
y = yposition
z = zposition
ax.plot(x, y, z, marker='D', linestyle='--', label="reference trajectory", color=[0.5,0,0], alpha=0.5, lw=2)

ax.set_xlabel('X')
ax.set_xlim(0, max(xi))
ax.set_ylabel('Y')
ax.set_ylim(0, max(yi))
ax.set_zlabel('Z')
ax.set_zlim(0, 4)

ax.legend(loc=2, prop={'size':30})
plt.show(block=True)

