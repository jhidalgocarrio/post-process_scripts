#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2017-02-27 15:52:50

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
# PLOTTING FUNCTION
##########################################################################
def adaptive_matches_comparison_figure(fig_num, info_ten, info_twentyfive, info_fifty,
        info_hundred, pred_mean, color_bar='Reds'):
    ########################
    # Plot matches
    ########################
    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
    fig, ax = plt.subplots()

    ########################
    # Odometry prediction 
    ########################
    x = info_ten.index.to_pydatetime()
    x[:] = [(i-x[0]).total_seconds() for i in x]
    y = info_ten.inliers_matches_ratio_th

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
    h_cbar = plt.colorbar(lc)#, orientation='horizontal')
    h_cbar.ax.set_ylabel(r'threshold - odometry error [%]', fontsize=25, fontweight='bold', color='k')

    ########################
    #Ten fill
    x = info_ten.index.to_datetime()
    y = info_ten.inliers_matches_ratio_th
    d = scipy.zeros(len(x))
    ax.fill_between(x, y, 0, color='lightblue')

    #Scatters
    scatter_ten = ax.scatter(x[0], y[0], marker='s', facecolor='lightblue',
            edgecolor='k', label='10%', s=20, alpha=1.0, zorder=100)

    #Twenty five fill
    x = info_twentyfive.index.to_datetime()
    y = info_twentyfive.inliers_matches_ratio_th
    d = scipy.zeros(len(x))
    ax.fill_between(x, y, 0, color='lightsteelblue')

    #Scatters
    scatter_twentyfive = ax.scatter(x[0], y[0], marker='s', facecolor='lightsteelblue',
            edgecolor='k', label='25%', s=20, alpha=1.0, zorder=100)

    #Fifty fill
    x = info_fifty.index.to_datetime()
    y = info_fifty.inliers_matches_ratio_th
    d = scipy.zeros(len(x))
    #ax.fill_between(x, y, 0, color='steelblue')

    #Scatters
    #scatter_fifty = ax.scatter(x[0], y[0], marker='s', facecolor='steelblue',
    #        edgecolor='k', label='50%', s=20, alpha=1.0, zorder=100)

    #Hundred fill
    x = info_hundred.index.to_datetime()
    y = info_hundred.inliers_matches_ratio_th
    d = scipy.zeros(len(x))
    ax.fill_between(x, y, 0, color='blue')

    #Scatters
    scatter_hundred = ax.scatter(x[0], y[0], marker='s', facecolor='blue',
            edgecolor='k', label='100%', s=20, alpha=1.0, zorder=100)

    ########################
    #Hundred line
    x = info_hundred.index.to_datetime()
    y = info_hundred.inliers_matches_ratio_th
    ax.plot(x, y, linestyle='-', lw=2, alpha=1.0, color=[0.0, 0.0, 0.0])

    #Fifty line
    x = info_fifty.index.to_datetime()
    y = info_fifty.inliers_matches_ratio_th
    #ax.plot(x, y, linestyle='-', lw=2, alpha=1.0, color=[0.0, 0.0, 0.0])

    #Twenty line
    x = info_twentyfive.index.to_datetime()
    y = info_twentyfive.inliers_matches_ratio_th
    ax.plot(x, y, linestyle='-', lw=2, alpha=1.0, color=[0.0, 0.0, 0.0])

    #Ten line
    x = info_ten.index.to_datetime()
    y = info_ten.inliers_matches_ratio_th
    ax.plot(x, y, linestyle='-', lw=2, alpha=1.0, color=[0.0, 0.0, 0.0])

    ax.set_ylabel(r'inliers matches ratio [$0.0 - 0.75$]', fontsize=25, fontweight='bold', color='k')
    #ax.tick_params('y', colors='k')

    ax.set_xlabel(r'Time', fontsize=25, fontweight='bold')
    #ax.tick_params('x', colors='k')
    plt.grid(True)
    plt.legend(handles=[scatter_ten, scatter_twentyfive, scatter_hundred],
            loc=2, prop={'size':15})
    plt.show(block=True)

def adaptive_frames_comparison_figure(fig_num, info_ten, info_twentyfive, info_fifty,
        info_hundred, pred_mean, color_bar='Reds'):

    ########################
    # Plot matches
    ########################
    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
    fig, ax = plt.subplots()

    ########################
    # Odometry prediction 
    ########################
    x = info_ten.index.to_pydatetime()
    x[:] = [(i-x[0]).total_seconds() for i in x]
    y = info_ten.desired_fps

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
    h_cbar = plt.colorbar(lc)#, orientation='horizontal')
    h_cbar.ax.set_ylabel(r'threshold - odometry error [%]', fontsize=25, fontweight='bold', color='k')

    ########################
    #Ten fill
    x = info_ten.index.to_datetime()
    y = info_ten.desired_fps
    d = scipy.zeros(len(x))
    ax.fill_between(x, y, 0, color='lightgreen')

    #Scatters
    scatter_ten = ax.scatter(x[0], y[0], marker='s', facecolor='lightgreen',
            edgecolor='k', label='10%', s=20, alpha=1.0, zorder=100)

    #Twenty five fill
    x = info_twentyfive.index.to_datetime()
    y = info_twentyfive.desired_fps
    d = scipy.zeros(len(x))
    ax.fill_between(x, y, 0, color='mediumseagreen')

    #Scatters
    scatter_twentyfive = ax.scatter(x[0], y[0], marker='s', facecolor='mediumseagreen',
            edgecolor='k', label='25%', s=20, alpha=1.0, zorder=100)

    #Fifty fill
    x = info_fifty.index.to_datetime()
    y = info_fifty.desired_fps
    d = scipy.zeros(len(x))
    #ax.fill_between(x, y, 0, color='forestgreen')

    #Scatters
    #scatter_fifty = ax.scatter(x[0], y[0], marker='s', facecolor='forestgreen',
    #        edgecolor='k', label='50%', s=20, alpha=1.0, zorder=100)

    #Hundred fill
    x = info_hundred.index.to_datetime()
    y = info_hundred.desired_fps
    d = scipy.zeros(len(x))
    ax.fill_between(x, y, 0, color='darkgreen')

    #Scatters
    scatter_hundred = ax.scatter(x[0], y[0], marker='s', facecolor='darkgreen',
            edgecolor='k', label='100%', s=20, alpha=1.0, zorder=100)

    ########################
    #Hundred line
    x = info_hundred.index.to_datetime()
    y = info_hundred.desired_fps
    ax.plot(x, y, linestyle='-', lw=2, alpha=1.0, color=[0.0, 0.0, 0.0])

    #Fifty line
    x = info_fifty.index.to_datetime()
    y = info_fifty.desired_fps
    #ax.plot(x, y, linestyle='-', lw=2, alpha=1.0, color=[0.0, 0.0, 0.0])

    #Twenty line
    x = info_twentyfive.index.to_datetime()
    y = info_twentyfive.desired_fps
    ax.plot(x, y, linestyle='-', lw=2, alpha=1.0, color=[0.0, 0.0, 0.0])

    #Ten line
    x = info_ten.index.to_datetime()
    y = info_ten.desired_fps
    ax.plot(x, y, linestyle='-', lw=2, alpha=1.0, color=[0.0, 0.0, 0.0])

    ax.set_ylabel(r'fps [$0.5 - 2.5$]', fontsize=25, fontweight='bold', color='k')
    #ax.tick_params('y', colors='k')

    ax.set_xlabel(r'Time', fontsize=25, fontweight='bold')
    #ax.tick_params('x', colors='k')
    plt.grid(True)
    plt.legend(handles=[scatter_ten, scatter_twentyfive, scatter_hundred],
            loc=2, prop={'size':15})
    plt.show(block=True)

##########################################################################
# Comparing information from different adaptivity thresholds
##########################################################################
path='~/npi/data/20141024_planetary_lab/20141027-2034_orb_slam2_quadratic_adaptivity_10/'
path_task_info_adaptive_slam_file = path + 'task_info_adaptive_slam.0.data'

# Information task
info_ten = pandas.read_csv(os.path.expanduser(path_task_info_adaptive_slam_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'relocalization', 'loops', 'icounts', 'desired_fps',
        'actual_fps', 'inliers_matches_ratio_th', 'map_matches_ratio_th',
        'inliers_matches_th', 'map_matches_ratio_cu',
        'inliers_matches_cu', 'frame_gp_residual', 'kf_gp_residual',
        'kf_gp_threshold', 'distance_traversed'], header=None)

path='~/npi/data/20141024_planetary_lab/20141027-2034_orb_slam2_quadratic_adaptivity_25/'
path_task_info_adaptive_slam_file = path + 'task_info_adaptive_slam.0.data'

# Information task
info_twentyfive = pandas.read_csv(os.path.expanduser(path_task_info_adaptive_slam_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'relocalization', 'loops', 'icounts', 'desired_fps',
        'actual_fps', 'inliers_matches_ratio_th', 'map_matches_ratio_th',
        'inliers_matches_th', 'map_matches_ratio_cu',
        'inliers_matches_cu', 'frame_gp_residual', 'kf_gp_residual',
        'kf_gp_threshold', 'distance_traversed'], header=None)

path='~/npi/data/20141024_planetary_lab/20141027-2034_orb_slam2_quadratic_adaptivity_50/'
path_task_info_adaptive_slam_file = path + 'task_info_adaptive_slam.0.data'

# Information task
info_fifty = pandas.read_csv(os.path.expanduser(path_task_info_adaptive_slam_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'relocalization', 'loops', 'icounts', 'desired_fps',
        'actual_fps', 'inliers_matches_ratio_th', 'map_matches_ratio_th',
        'inliers_matches_th', 'map_matches_ratio_cu',
        'inliers_matches_cu', 'frame_gp_residual', 'kf_gp_residual',
        'kf_gp_threshold', 'distance_traversed'], header=None)

path='~/npi/data/20141024_planetary_lab/20141027-2034_orb_slam2_quadratic_adaptivity_100/'
path_task_info_adaptive_slam_file = path + 'task_info_adaptive_slam.0.data'

# Information task
info_hundred = pandas.read_csv(os.path.expanduser(path_task_info_adaptive_slam_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'relocalization', 'loops', 'icounts', 'desired_fps',
        'actual_fps', 'inliers_matches_ratio_th', 'map_matches_ratio_th',
        'inliers_matches_th', 'map_matches_ratio_cu',
        'inliers_matches_cu', 'frame_gp_residual', 'kf_gp_residual',
        'kf_gp_threshold', 'distance_traversed'], header=None)

##########################################################################
path_gpy_gaussian_process_model_file = '~/npi/dev/bundles/exoter/data/gaussian_processes/SparseGP_RBF_NL_xyz_velocities_train_at_1s_normalized_exoter_odometry_arl_residuals_20141027-2034.data'
##########################################################################
path = '~/npi/data/20141024_planetary_lab/20141027-2034/'
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
#info = info.resample(resampling_time).mean()

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

##########################################################################
# PLOT
##########################################################################
adaptive_matches_comparison_figure(1, info_ten, info_twentyfive, info_fifty, info_hundred, pred_mean)
adaptive_frames_comparison_figure(1, info_ten, info_twentyfive, info_fifty, info_hundred, pred_mean)

