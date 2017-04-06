#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2017-02-28 14:05:29

#######################################
path='~/npi/data/20150515_planetary_lab/20150515-1752_no_adaptive_slam/'
#path='~/npi/data/20150515_planetary_lab/20150515-1752_orb_slam2_quadratic_adaptivity_25_bis/'
#######################################
path_odometry_file = path + 'pose_odo_position.0.data'

path_reference_file = path + 'pose_ref_position.0.data'

path_delta_reference_file = path + 'delta_pose_ref_position.0.data'

path_orb_slam2_position_file = path + 'pose_orb_slam2_position.0.data'

path_keyframes_position_file = path + 'pose_keyframe_orb_slam2_position.0.data'

path_keyframes_trajectory_file = path + 'orb_slam2_keyframes_trajectory.data'

path_allframes_trajectory_file = path + 'orb_slam2_allframes_trajectory.data'

path_orb_slam2_information_file = path + 'task_orb_slam2_info.0.data'

path_navigation_orientation_file = path + 'pose_world_to_navigation_orientation.0.data'

path_navigation_position_file = path + 'pose_world_to_navigation_position.0.data'

path_task_info_adaptive_slam_file = path + 'task_info_adaptive_slam.0.data'

#######################################
esa_arl_dem_file = '~/npi/documentation/esa_terrain_lab/DEMclean.ply'
#######################################
path_gpy_gaussian_process_model_file = '~/npi/dev/bundles/exoter/data/gaussian_processes/SparseGP_RBF_NL_xyz_velocities_train_at_1s_normalized_exoter_odometry_arl_residuals_20150515-1752.data'
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
matplotlib.style.use('ggplot') #in matplotlib >= 1.5.1
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
def arl_dem_figure(fig_num, dem_file, trajectory, pred_mean, kf_trajectory, frames_trajectory, color_bar='Reds'):

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


    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
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

    # Display Ground Truth trajectory
    from numpy import linalg as la
    x = trajectory[1:trajectory.shape[0]-1,0]
    y = trajectory[1:trajectory.shape[0]-1,1]
    sd = la.norm(pred_mean, axis=1)
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    from matplotlib.collections import LineCollection
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.colors import LinearSegmentedColormap as lscm

    cmap = plt.get_cmap(color_bar)

    norm = plt.Normalize(0.00, 0.0634491701615)
    lc = LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(sd)
    lc.set_linewidth(40)
    lc.set_alpha(0.8)
    plt.gca().add_collection(lc)


    #color bar of the covarianve
    #h_cbar = plt.colorbar(lc)#, orientation='horizontal')
    #h_cbar.ax.set_ylabel(r' odometry error[$m/s$]',  fontsize=25, fontweight='bold')

    # Color bar of the dem
    #cbar = plt.colorbar()  # draw colorbar
    #cbar.ax.set_ylabel(r' terrain elevation[$m$]', fontsize=25, fontweight='bold')

    # Plot all the image frames line
    fr_x = frames_trajectory[:,0]
    fr_y = frames_trajectory[:,1]
    ax.plot(fr_x, fr_y, linestyle='-', lw=2, alpha=0.4, color=[0.0, 0.3, 1.0],
            label='slam trajectory',
            zorder=98)
    # Plot all the image frames
    ax.scatter(fr_x, fr_y, marker='s', facecolor=[0.0,0.3,1.0], edgecolor='b',
            label='image frames', s=80, alpha=0.3, zorder=99)


    # Plot the key frames
    kf_x = kf_trajectory[:,0]
    kf_y = kf_trajectory[:,1]
    ax.scatter(kf_x, kf_y, marker='D', facecolor=[0.2,1.0,0.0], edgecolor='b',
            label='keyframes', s=80, alpha=1.0, zorder=100)

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

    ax.arrow(x[0], y[0], x[64]-x[0], y[64]-y[0], width=0.01, head_width=0.05,
            head_length=0.05, fc='k', ec='k', zorder=104)

    # End sign
    ax.annotate(r'End', xy=(fr_x[fr_x.shape[0]-1], fr_y[fr_y.shape[0]-1]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points', fontsize=12,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(fr_x[fr_x.shape[0]-1], fr_y[fr_y.shape[0]-1], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)

    ax.add_artist(ab)

    plt.xlabel(r'X [$m$]', fontsize=15, fontweight='bold')
    plt.ylabel(r'Y [$m$]', fontsize=15, fontweight='bold')
    #ax.legend(loc=2, prop={'size':15})
    #plt.axis('equal')
    plt.grid(True)
    fig.savefig("arl_adaptive_slam_dem_20150515-1752.png", dpi=fig.dpi)
    plt.show(block=True)

def arl_trajectories_figure(fig_num, dem_file, reference_trajectory, kf_trajectory, frames_trajectory, odo_trajectory):
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


    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
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
    cbar.ax.set_ylabel(r' terrain elevation[$m$] ',  fontsize=25, fontweight='bold')

    # Display Ground Truth trajectory
    from numpy import linalg as la
    x = reference_trajectory[1:reference_trajectory.shape[0]-1,0][0::5]
    y = reference_trajectory[1:reference_trajectory.shape[0]-1,1][0::5]
    ax.plot(x, y, marker='D', linestyle='-', lw=2, alpha=0.3, color=[1.0, 1.0, 0.0],
            label='ground truth', zorder=80)

    # Plot all the image frames
    fr_x = frames_trajectory[:,0][0::5]
    fr_y = frames_trajectory[:,1][0::5]
    ax.plot(fr_x, fr_y, marker='s', linestyle='-', lw=2, alpha=0.3, color=[0.0, 0.3, 1.0],
            label='slam', zorder=99)

    # Plot the key frames
    kf_x = kf_trajectory[:,0]
    kf_y = kf_trajectory[:,1]
    ax.scatter(kf_x, kf_y, marker='s', facecolor=[0.2,1.0,0.0], edgecolor='b',
            label='keyframes', s=20, alpha=1.0, zorder=100)

    # Pure odometry trajectory
    odo_x = odo_trajectory[:,0][0::5]
    odo_y = odo_trajectory[:,1][0::5]
    ax.plot(odo_x, odo_y, marker='h', linestyle='-', lw=2, alpha=0.3, color=[1.0, 0, 0],
            label='odometry', zorder=70)

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
                            xytext=(-20, 30), textcoords='offset points', fontsize=12,
                            )
    ax.annotate(r'Start', xy=(x[0], y[0]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points', fontsize=12,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[0], y[0], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)

    ax.arrow(x[0], y[0], x[13]-x[0], y[13]-y[0], width=0.01, head_width=0.05,
            head_length=0.05, fc='k', ec='k', zorder=104)

    ax.annotate(r'End', xy=(x[x.shape[0]-1], y[y.shape[0]-1]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points', fontsize=12,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[x.shape[0]-1], y[y.shape[0]-1], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)

    ax.add_artist(ab)

    plt.xlabel(r'X [$m$]', fontsize=15, fontweight='bold')
    plt.ylabel(r'Y [$m$]', fontsize=15, fontweight='bold')
    ax.legend(loc=1, prop={'size':15})
    plt.grid(True)
    fig.savefig("arl_trajectories_figure_20150515-1752.png", dpi=fig.dpi)
    plt.show(block=True)

def adaptive_matches_figure(fig_num, info):
    ########################
    # Plot matches 
    ########################
    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
    fig, ax = plt.subplots()

    x = info.index.to_datetime()
    y = info.inliers_matches_ratio_th
    ax.plot(x, y, linestyle='--', lw=2, alpha=1.0, color=[1.0, 0, 0.0])

    d = scipy.zeros(len(x))
    ax.fill_between(x, y, 0, color='blue')

    ax.set_ylabel(r'Inliers Ratio[$0.0 - 0.7 $]', fontsize=25, fontweight='bold', color='k')
    ax.tick_params('y', colors='k')

    ax.set_xlabel(r'Time', fontsize=25, fontweight='bold')
    ax.tick_params('x', colors='k')
    plt.grid(True)
    plt.legend(loc=1, prop={'size':15})
    plt.show(block=True)

def adaptive_frame_figure(fig_num, info):
    ########################
    # Plot matches 
    ########################
    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
    fig, ax = plt.subplots()

    x = info.index.to_datetime()
    y = info.desired_fps
    ax.plot(x, y, linestyle='--', lw=2, alpha=1.0, color=[1.0, 0, 0.0])

    d = scipy.zeros(len(x))
    ax.fill_between(x, y, 0, color='blue')

    ax.set_ylabel(r'Image fps [$Hz$]', fontsize=25, fontweight='bold', color='k')
    ax.tick_params('y', colors='k')

    ax.set_xlabel(r'Time', fontsize=25, fontweight='bold')
    ax.tick_params('x', colors='k')
    plt.grid(True)
    plt.legend(loc=1, prop={'size':15})
    plt.show(block=True)

def odometry_error_bar(fig_num, info, pred_mean,  color_bar='Reds'):
    ########################
    # Plot Bar 
    ########################
    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
    fig, ax = plt.subplots()

    x = info.index.to_pydatetime()
    x[:] = [(i-x[0]).total_seconds() for i in x]
    y = info.inliers_matches_ratio_th

    # Display Odometry error information
    from numpy import linalg as la
    y  = np.ones(len(x))
    sd = la.norm(pred_mean, axis=1)
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    from matplotlib.collections import LineCollection
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.colors import LinearSegmentedColormap as lscm

    cmap = plt.get_cmap(color_bar)

    norm = plt.Normalize(0.00, 0.0634491701615)
    lc = LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(sd)
    lc.set_linewidth(100)
    lc.set_alpha(0.8)
    plt.gca().add_collection(lc)

    ax.tick_params('y', colors='k')
    ax.set_xlabel(r'Time [$s$]', fontsize=25, fontweight='bold')
    ax.tick_params('x', colors='k')
    ax.set_xlim([0.5, 1.5])
    ax.set_ylim([x[0], x[len(x)-1]])
    plt.show(block=True)

##########################################################################
# READ THE VALUES IN PANDAS
##########################################################################
#ExoTeR Odometry
odometry = pandas.read_csv(os.path.expanduser(path_odometry_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)


#Reference Position
reference = pandas.read_csv(os.path.expanduser(path_reference_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#ExoTeR Delta Odometry
delta_reference = pandas.read_csv(os.path.expanduser(path_delta_reference_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#ORB_SLAM2 Position
orb_slam2 = pandas.read_csv(os.path.expanduser(path_orb_slam2_position_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#orb_slam2 = orb_slam2[['x', 'y', 'z']].copy()

# KeyFrames Trajectory
keyframes = pandas.read_csv(os.path.expanduser(path_keyframes_trajectory_file), sep=" ", parse_dates=False,
    names=['x', 'y', 'z', 'heading'], header=None)

# Images Frames Trajectory
imageframes = pandas.read_csv(os.path.expanduser(path_allframes_trajectory_file), sep=" ", parse_dates=False,
    names=['x', 'y', 'z', 'heading'], header=None)

# Information task
info = pandas.read_csv(os.path.expanduser(path_task_info_adaptive_slam_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'relocalization', 'loops', 'icounts', 'desired_fps',
        'actual_fps', 'inliers_matches_ratio_th', 'map_matches_ratio_th',
        'inliers_matches_th', 'map_matches_ratio_cu',
        'inliers_matches_cu', 'frame_gp_residual', 'kf_gp_residual',
        'kf_gp_threshold', 'distance_traversed'], header=None)

#World to Navigation Pose
navigation_orient = data.QuaternionData()
navigation_orient.readData(os.path.expanduser(path_navigation_orientation_file), cov=False)
navigation_position = data.ThreeData()
navigation_position.readData(os.path.expanduser(path_navigation_position_file), cov=False)

##########################################################################
# CREATE THE GP DATASET
##########################################################################

###################
## PREDICTION    ##
###################
path = '~/npi/data/20150515_planetary_lab/20150515-1752/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

pose_ref_velocity_file =  path + 'delta_pose_ref_velocity.0.data'

pose_odo_velocity_file =  path + 'delta_pose_odo_velocity.0.data'

pose_imu_orientation_file =  path + 'pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data'
##########################################################################
# READ THE VALUES IN PANDAS
##########################################################################

# Reference Robot Velocity
reference_velocity = pandas.read_csv(os.path.expanduser(pose_ref_velocity_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

# Odometry Robot Velocity
odometry_velocity = pandas.read_csv(os.path.expanduser(pose_odo_velocity_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

# IMU orientation (in quaternion form)
data_orient = data.QuaternionData()
data_orient.readData(os.path.expanduser(pose_imu_orientation_file), cov=True)
time = [dateparse(x*1e06) for x in data_orient.atime]
euler = np.column_stack((data_orient.getEuler(2), data_orient.getEuler(1), data_orient.getEuler(0)))
imu_orient = pandas.DataFrame(data=euler, columns=['x', 'y', 'z'], index=time)

# IMU acceleration
imu_acc = pandas.read_csv(os.path.expanduser(pose_imu_acceleration_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z'], header=None)

# IMU Angular Velocity
imu_gyro = pandas.read_csv(os.path.expanduser(pose_imu_angular_velocity_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z'], header=None)

# Robot Joints Position and Speed
names = ["time", "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"]

joints_position = pandas.read_csv(os.path.expanduser(joints_position_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time', names=names, header=None)

joints_speed = pandas.read_csv(os.path.expanduser(joints_speed_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time', names=names, header=None)

##########################################################################
# REMOVE OUTLIERS
##########################################################################
# ExoTer Max velocity is 10cm/s
reference_velocity = reference_velocity.drop(reference_velocity[fabs(reference_velocity.x) > 0.15].index)
odometry_velocity = odometry_velocity.drop(odometry_velocity[fabs(odometry_velocity.x) > 0.15].index)

#################
## RE-SAMPLE   ##
#################
resampling_time = '1s'
reference_velocity = reference_velocity.resample(resampling_time).mean()
odometry_velocity = odometry_velocity.resample(resampling_time).mean()
imu_orient = imu_orient.resample(resampling_time).mean()
imu_acc = imu_acc.resample(resampling_time).mean()
imu_gyro = imu_gyro.resample(resampling_time).mean()
joints_position = joints_position.resample(resampling_time).mean()
joints_speed = joints_speed.resample(resampling_time).mean()

#Compute the error in odometry
odometry_velocity['error_x'] = pandas.Series (fabs(odometry_velocity.x - reference_velocity.x))
odometry_velocity['error_y'] = pandas.Series (fabs(odometry_velocity.y - reference_velocity.y))
odometry_velocity['error_z'] = pandas.Series (fabs(odometry_velocity.z - reference_velocity.z))

#Compute the error in reference
reference_velocity['error_x'] = pandas.Series (fabs(reference_velocity.x - odometry_velocity.x))
reference_velocity['error_y'] = pandas.Series (fabs(reference_velocity.y - odometry_velocity.y))
reference_velocity['error_z'] = pandas.Series (fabs(reference_velocity.z - odometry_velocity.z))

##########################################################################
# ELIMINATE NULL VALUES
##########################################################################
test_mask = pandas.notnull(odometry_velocity.error_x) & pandas.notnull(odometry_velocity.error_y) & pandas.notnull(odometry_velocity.error_z)

##########################################################################
# GAUSSIAN PROCESS X INPUT VECTOR
##########################################################################
Xp_orientation = np.column_stack((imu_orient.x.as_matrix(),
                imu_orient.y.as_matrix()))

Xp_joints_position = np.column_stack((
                joints_position.left_passive.as_matrix(),
                joints_position.right_passive.as_matrix(),
                joints_position.rear_passive.as_matrix(),
                joints_position.fl_steer.as_matrix(),
                joints_position.fr_steer.as_matrix(),
                joints_position.rl_steer.as_matrix(),
                joints_position.rr_steer.as_matrix(),
                ))

Xp_joints_speed = np.column_stack((
                joints_speed.left_passive.as_matrix(),
                joints_speed.right_passive.as_matrix(),
                joints_speed.rear_passive.as_matrix(),
                joints_speed.fl_steer.as_matrix(),
                joints_speed.fr_steer.as_matrix(),
                joints_speed.rl_steer.as_matrix(),
                joints_speed.rr_steer.as_matrix(),
                joints_speed.fl_translation.as_matrix(),
                joints_speed.fr_translation.as_matrix(),
                joints_speed.ml_translation.as_matrix(),
                joints_speed.mr_translation.as_matrix(),
                joints_speed.rl_translation.as_matrix(),
                joints_speed.rr_translation.as_matrix(),
                ))

Xp_acc =  np.column_stack((
                imu_acc.x.as_matrix(),
                imu_acc.y.as_matrix(),
                imu_acc.z.as_matrix()
                ))

Xp_gyro =  np.column_stack((
                imu_gyro.x.as_matrix(),
                imu_gyro.y.as_matrix(),
                imu_gyro.z.as_matrix()
                ))



Xp = np.column_stack((Xp_orientation, Xp_joints_position, Xp_joints_speed,
    Xp_acc, Xp_gyro))

##########################################################################
# READ THE GP_MODEL
##########################################################################
m = data.open_object(path_gpy_gaussian_process_model_file)
[pred_mean, pred_var] = m.predict(Xp)

#################
## RE-SAMPLE   ##
#################
reference = reference.resample(resampling_time).mean()
odometry = odometry.resample(resampling_time).mean()
orb_slam2 = orb_slam2.resample(resampling_time).pad()

##########################################################################
#rotate and translate the reference trajectory wrt the world frame
##########################################################################
reference_position = np.column_stack((reference.x.values, reference.y.values,  reference.z.values ))
reference_position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in reference_position]

##########################################################################
#rotate and translate the odometry trajectory wrt the world frame
##########################################################################
odometry_position = np.column_stack((odometry.x.values, odometry.y.values,  odometry.z.values ))
odometry_position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in odometry_position]

##########################################################################
#rotate and translate the slam trajectory wrt the world frame
##########################################################################
slam_position = np.column_stack((orb_slam2.x.values, orb_slam2.y.values,  orb_slam2.z.values ))
slam_position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in slam_position]

##########################################################################
#rotate and translate the keyframes trajectory wrt the world frame
##########################################################################
keyframes_position = np.column_stack((keyframes.x.values, keyframes.y.values,  keyframes.z.values ))
keyframes_position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in keyframes_position]

##########################################################################
#rotate and translate the image frames trajectory wrt the world frame
##########################################################################
imframes_position = np.column_stack((imageframes.x.values, imageframes.y.values,  imageframes.z.values ))
imframes_position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in imframes_position]

##########################################################################
arl_trajectories_figure(1, esa_arl_dem_file, reference_position, keyframes_position, imframes_position, odometry_position)
arl_dem_figure(2, esa_arl_dem_file, reference_position, pred_mean, keyframes_position, imframes_position)
##########################################################################
adaptive_matches_figure(3, info)
adaptive_frame_figure(4, info)

info = info.resample(resampling_time).mean()
odometry_error_bar(5, info, pred_mean)
##########################################################################
# Compute RMSE, FINAL ERROR AND MAXIMUM ERROR
##########################################################################
from evaluation import RMSE, MAE, MAPE

#ORB_SLAM2 Keyframes Position
kf_orb_slam2 = pandas.read_csv(os.path.expanduser(path_keyframes_position_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

estimation = orb_slam2.resample('1s').pad()
ground_truth = reference.resample('1s').pad()
##########################################################################

rmse = RMSE()
eval_rmse = rmse.evaluate(estimation, ground_truth)
la.norm(eval_rmse) #0.145m with adaptation # 0.163m original #1.827m 0.5fps w/relocalization # 0.204 m w/o relocalization
print("RMSE: " + str(la.norm(eval_rmse[0:3])))
##########################################################################
final_estimation = np.array([estimation.x[estimation.shape[0]-1], estimation.y[estimation.shape[0]-1], estimation.z[estimation.shape[0]-1]])
final_groundtruth = np.array([ground_truth.x[ground_truth.shape[0]-1], ground_truth.y[ground_truth.shape[0]-1], ground_truth.z[ground_truth.shape[0]-1]])

final_error = final_estimation - final_groundtruth

la.norm(final_error) #0.264m #0.264m original #0.52m 0.5fps
print("Final error: " + str(la.norm(final_error)))

##########################################################################
max_error = np.max(estimation - ground_truth)
la.norm(max_error) #0.468m adaptation 0.4553m original #4.620m 0.5fps w/relocalization #0.729m w/o relocalization
print("Maximum error: " + str(la.norm(max_error[0:3])))
##########################################################################
# Number of Keyframes and frames
##########################################################################
number_keyframes = keyframes.shape[0] # 135 (adaptation) 181(original) 82(0.5fps w/relocalization)  150(0.5fps w/o relocalization)keyframes 
number_frames = imageframes.shape[0] # 484 (adaptation) 2582 (original) 343+169 (0.5fps w/relocalization) 500 (0.5fps w/o relocalization) images frames
print("#Keyframes: " + str(number_keyframes))
print("#Frames: " + str(number_frames))
print("#Loops: " + str(max(info.loops)))
