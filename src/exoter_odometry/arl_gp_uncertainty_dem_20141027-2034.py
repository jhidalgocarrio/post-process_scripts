#! /usr/bin/env python
#path ='/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034_dem_coloring_with_gp_with_sam/'
path='/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034_threed_odometry_with_gp/'
#######################################
path_odometry_file = path + 'pose_odo_position.0.data'

path_reference_file = path + 'pose_ref_position.0.data'

path_navigation_orientation_file = path + 'pose_world_to_navigation_orientation.0.data'

path_navigation_position_file = path + 'pose_world_to_navigation_position.0.data'

path_gp_odo_delta_position_file = path + 'delta_pose_gp_odo_position.0.data'

path_gp_odo_velocity_file = path + 'delta_pose_gp_odo_velocity.0.data'
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
from numpy import linalg as la

import pandas as pandas
import datetime
pandas.set_option('display.mpl_style', 'default') # Make the graphs a bit prettier
def dateparse (time_in_microsecs):
    return datetime.datetime.fromtimestamp(float(time_in_microsecs * 1e-06))

##########################################################################
# READ THE VALUES IN PANDAS
##########################################################################

#ExoTeR Odometry
odometry = pandas.read_csv(path_odometry_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

# Odometry Robot Velocity
gp_odometry_velocity = data.ThreeData()
gp_odometry_velocity.readData(path_gp_odo_velocity_file, cov=True)
gp_odometry_velocity = pandas.read_csv(path_gp_odo_velocity_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#Reference Pose
reference = pandas.read_csv(path_reference_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)


#Reference Velocity
reference_velocity = pandas.read_csv(path_reference_velocity_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)



#World to Navigation Pose
navigation_orient = data.QuaternionData()
navigation_orient.readData(path_navigation_orientation_file, cov=False)
navigation_position = data.ThreeData()
navigation_position.readData(path_navigation_position_file, cov=False)


########################
### REMOVE OUTLIERS  ###
########################
reference_velocity = reference_velocity.drop(reference_velocity[fabs(reference_velocity.x) > 0.15].index)
gp_odometry_velocity =gp_odometry_velocity.drop(gp_odometry_velocity[fabs(gp_odometry_velocity.x) > 0.15].index)


########################
## CONVOLUTE FILTER   ##
########################
#delta = (reference_velocity.index[14] - reference_velocity.index[13])
#sampling_frequency = 1.0/delta.total_seconds()
#size_block = 1 * sampling_frequency
#window = np.ones(size_block)
#window /= sum(window)
#
#reference_velocity.x = np.convolve(reference_velocity.x, window, mode='same')
#reference_velocity.y = np.convolve(reference_velocity.y, window, mode='same')
#reference_velocity.z = np.convolve(reference_velocity.z, window, mode='same')
#
#gp_odometry_velocity.x = np.convolve(gp_odometry_velocity.x, window, mode='same')
#gp_odometry_velocity.y = np.convolve(gp_odometry_velocity.y, window, mode='same')
#gp_odometry_velocity.z = np.convolve(gp_odometry_velocity.z, window, mode='same')

########################
# Load Terrain DEM
########################
plydata = PlyData.read(open(esa_arl_dem_file))

vertex = plydata['vertex'].data

[px, py, pz] = (vertex[t] for t in ('x', 'y', 'z'))

# define grid.
npts=100
xi = np.linspace(min(px), max(px), npts)
yi = np.linspace(min(py), max(py), npts)

# grid the data.
zi = griddata(px, py, pz, xi, yi, interp='linear')

#################
## RE-SAMPLE   ##
#################
resampling_time = '1s'
reference = reference.resample(resampling_time)
reference_velocity = reference_velocity.resample(resampling_time)
odometry = odometry.resample(resampling_time)
gp_odometry_velocity = gp_odometry_velocity.resample(resampling_time)

########################################################
#rotate and translate the trajectory wrt the world frame
########################################################
position = np.column_stack((reference.x.values, reference.y.values,  reference.z.values ))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

########################
## GET THE ERROR
########################
variance = np.column_stack((gp_odometry_velocity.cov_xx, gp_odometry_velocity.cov_yy, gp_odometry_velocity.cov_zz))

############
### PLOT ###
############
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(3)
ax = fig.add_subplot(111)

# Display the DEM
plt.rc('text', usetex=False)# activate latex text rendering
CS = plt.contour(xi, yi, zi, 15, linewidths=0.5, colors='k')
CS = plt.contourf(xi, yi, zi, 15, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())
# plot data points.
plt.xlim(min(px), max(xi))
plt.ylim(min(py), max(yi))


# Display Ground Truth trajectory
x = position[:,0]
y = position[:,1]
sd = la.norm(np.sqrt(variance), axis=1)
points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.colors import LinearSegmentedColormap as lscm
cmap = plt.get_cmap('Greens')
#cmap = lscm.from_list('temp', colors)
norm = plt.Normalize(min(sd), max(sd))
lc = LineCollection(segments, cmap=cmap, norm=norm)
lc.set_array(sd)
lc.set_linewidth(40)
lc.set_alpha(0.8)
plt.gca().add_collection(lc)


#color bar of the covariamve
#cbaxes = fig.add_axes([0.8, 0.1, 0.03, 0.8]) 
h_cbar = plt.colorbar(lc)#, orientation='horizontal')
h_cbar.ax.set_ylabel(r' ground truth residual [$m/s$]')

# Color bar of the dem
cbar = plt.colorbar()  # draw colorbar
cbar.ax.set_ylabel(r' terrain elevation[$m$]')

#Q = ax.plot(x, y, marker='o', linestyle='-', color=[0.3,0.2,0.4], alpha=0.5, lw=40)

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
#plt.axis('equal')
plt.grid(True)
#ax.legend(handles=[exoter], loc=1, prop={'size':30})
plt.show(block=False)

