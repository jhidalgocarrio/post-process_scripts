#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2018-07-18 16:27:44
#######################################
path='~/npi/data/20140911_shark_slam/'
#######################################
path_reference_position_file = path + 'pose_ref_position.0.data'

path_reference_orientation_file = path + 'pose_ref_orientation.0.data'

path_shark_slam_position_file = path + 'pose_shark_slam_position.0.data'

path_shark_slam_orientation_file = path + 'pose_shark_slam_orientation.0.data'

path_navigation_orientation_file = path + 'pose_world_to_navigation_orientation.0.data'

path_navigation_position_file = path + 'pose_world_to_navigation_position.0.data'
#######################################
decos_dem_file ='~/npi/documentation/decos_terrain/decos_selected_testing_zoom_area_point_cloud.ply'
#######################################

import sys
sys.path.insert(0, './src/core')
sys.path.insert(0, './src/gaussian_process/benchmarks')
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
matplotlib.style.use('classic') #in matplotlib >= 1.5.1
#pandas.set_option('display.mpl_style', 'default') # Make the graphs a bit prettier
def dateparse (time_in_microsecs):
    return datetime.datetime.fromtimestamp(float(time_in_microsecs) * 1e-06)

from methods import GP_RBF, SVIGP_RBF, SparseGP_RBF, SparseGP_RBF_NL, GP_MAT32, SparseGP_MAT32, GP_MAT52, SparseGP_MAT52
##########################################################################
# ARROW FUNCTION
##########################################################################
def add_arrow(line,position = None,direction = 'right',size=15,color = None):
   """ add an arrow to a line.

       line: Line2D object
       position: x-position of the arrow. If None, mean of xdata is taken 
       direction:  'left' or 'right'
       size:  size of the arrow in fontsize points
       color: if None, line color is taken.
       """
   if color is None:
       color = line.get_color()

   xdata = line.get_xdata()
   ydata = line.get_ydata()

   if position is None:
       position = xdata.mean()
   # find closest index
   start_ind = np.argmin(np.absolute(xdata-position))
   if direction == 'right':
       end_ind = start_ind + 1
   else:
       end_ind = start_ind - 1

   line.axes.annotate('',xytext = (xdata[start_ind],ydata[start_ind]),xy = (xdata[end_ind],ydata[end_ind]),arrowprops=dict(arrowstyle="->",color = color),size = size)


##########################################################################
# PLOTTING FUNCTION
##########################################################################
def decos_trajectories_figure(fig_num, dem_file, reference_trajectory,
        slam_trajectory,
        mis_orient = quat.quaternion(np.array([1.0,0.0,0.0,0.0])),
        mis_position = [0.00, 0.00, 0.00]):
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

    #################################################
    # Misalignment all trajectories wrt to the map
    #################################################
    map_posi_align = [1.00, 5.00, 0.00]
    map_orient_align = quat.quaternion.fromAngleAxis(-25.0 * np.pi/180.0, [0.0, 0.0,1.0])

    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
    fig = plt.figure(fig_num, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.add_subplot(111)
    #fig, ax = plt.subplots()

    # Display the DEM
    plt.rc('text', usetex=False)# activate latex text rendering
    CS = plt.contour(dem_xi, dem_yi, dem_zi, 20, linewidths=0.5, colors='k')
    CS = plt.contourf(dem_xi, dem_yi, dem_zi, 20, cmap=plt.cm.gray, vmax=abs(dem_zi).max(), vmin=-abs(dem_zi).max())
    #plt.clabel(CS, inline=1, fontsize=10) # number in the contour line

    # plot data points.
    plt.xlim(min(dem_px), max(dem_xi))
    plt.ylim(min(dem_py), max(dem_yi))

    # Color bar of the dem
    cbar = plt.colorbar()  # draw colorbar
    cbar.ax.set_ylabel(r' terrain elevation[$m$] ',  fontsize=25, fontweight='bold')

    # Display Ground Truth trajectory
    from numpy import linalg as la
    ref = np.column_stack((reference_trajectory[:,0][0::10], reference_trajectory[:,1][0::10], reference_trajectory[:,2][0::10]))
    ref[:] = [(map_orient_align * i * map_orient_align.conj())[1:4] for i in ref ]
    ref[:] = [ i + map_posi_align for i in ref ]
    x = ref[:,0]
    y = ref[:,1]
    ax.plot(x, y, marker='D', linestyle='-', lw=2, alpha=0.3, color=[1.0, 1.0, 0.0],
            label='ground truth', zorder=80)

    # Plot the slam trajectory
    st = np.column_stack((slam_trajectory[:,0], slam_trajectory[:,1], slam_trajectory[:,2]))
    st[:] = [(mis_orient * i * mis_orient.conj())[1:4] for i in st]
    st[:] = [ i + mis_position for i in st ]
    st[:] = [(map_orient_align * i * map_orient_align.conj())[1:4] for i in st ]
    st[:] = [ i + map_posi_align for i in st ]
    st_x = st[:,0][0::10]
    st_y = st[:,1][0::10]
    ax.plot(st_x, st_y, linestyle='-', lw=2, alpha=0.7, color=[0.2, 1.0, 0.0], )
    ax.scatter(st_x, st_y, marker='s', facecolor=[0.2,1.0,0.0], edgecolor='b',
            label='shark_slam', s=20, alpha=1.0, zorder=100)

    import os
    from matplotlib.cbook import get_sample_data
    from matplotlib._png import read_png
    import matplotlib.image as image
    from scipy import ndimage
    from matplotlib.offsetbox import OffsetImage, AnnotationBbox

    ax.annotate(r'Start', xy=(x[0], y[0]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points', fontsize=16,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[0], y[0], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)

    #ax.arrow(x[0], y[0], x[13]-x[0], y[13]-y[0], width=0.01, head_width=0.05,
    #        head_length=0.2, fc='k', ec='k', zorder=104)

    ax.annotate(r'End', xy=(x[x.shape[0]-1], y[y.shape[0]-1]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points',
                            fontsize=16,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[x.shape[0]-1], y[y.shape[0]-1], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)


    plt.xlabel(r'X [$m$]', fontsize=15, fontweight='bold')
    plt.ylabel(r'Y [$m$]', fontsize=15, fontweight='bold')
    ax.legend(loc=1, prop={'size':15})
    plt.grid(True)
    fig.savefig("decos_trajectories_figure_20140911-1805.png", dpi=fig.dpi)
    plt.show(block=True)

##########################################################################
# READ THE VALUES IN PANDAS
##########################################################################

#Reference Position
reference = pandas.read_csv(os.path.expanduser(path_reference_position_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

# Read the reference orientation information
reference_orient = data.QuaternionData()
reference_orient.readData(path_reference_orientation_file, cov=True)

# SHARK_SLAM Position
shark_slam_position = pandas.read_csv(os.path.expanduser(path_shark_slam_position_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
   # names=['time', 'x', 'y', 'z'], header=None)
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

# SHARK_SLAM Orientation (in quaternion form)
shark_slam_orient = data.QuaternionData()
shark_slam_orient.readData(os.path.expanduser(path_shark_slam_orientation_file), cov=True)

#World to Navigation Pose
navigation_orient = data.QuaternionData()
navigation_orient.readData(os.path.expanduser(path_navigation_orientation_file), cov=False)
navigation_position = data.ThreeData()
navigation_position.readData(os.path.expanduser(path_navigation_position_file), cov=False)


#Eliminate duplicated
shark_slam_position = shark_slam_position[~shark_slam_position.index.duplicated(keep='first')]

#################
## RE-SAMPLE   ##
#################
resampling_time = '1s'
reference = reference.resample(resampling_time).mean()
slam_position = shark_slam_position.resample(resampling_time).pad()

##########################################################################
#rotate and translate the reference trajectory wrt the world frame
##########################################################################
reference_position = np.column_stack((reference.x.values, reference.y.values,  reference.z.values ))
reference_position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in reference_position]

##########################################################################
#rotate and translate the slam trajectory wrt the world frame
##########################################################################
slam_position = np.column_stack((slam_position.x.values, slam_position.y.values, slam_position.z.values ))
slam_position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in slam_position]

#################################################
# Take the misalignment between odo/slam and gt
#################################################
mis_orient = quat.quaternion.fromAngleAxis(-0.0 * np.pi/180.0, [0.0, 0.0, 1.0])
mis_position = [0.0, 0.0, 0.00]

##########################################################################
decos_trajectories_figure(1, decos_dem_file, reference_position, slam_position, mis_orient, mis_position)
##########################################################################

