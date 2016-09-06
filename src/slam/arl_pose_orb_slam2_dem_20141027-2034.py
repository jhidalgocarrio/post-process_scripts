#! /usr/bin/env python
# by javi 2016-09-06 15:15:16

#######################################
path='/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034_orb_slam2_gp_adaptive_first_test/'
#######################################
path_odometry_file = path + 'pose_odo_position.0.data'

path_reference_file = path + 'pose_ref_position.0.data'

path_orb_slam2_position_file = path + 'pose_orb_slam2_position.0.data'

path_keyframes_trajectory_file = path + 'orb_slam2_keyframes_trajectory.data'

path_allframes_trajectory_file = path + 'orb_slam2_allframes_trajectory.data'

path_orb_slam2_information_file = path + 'task_orb_slam2_info.0.data'

path_navigation_orientation_file = path + 'pose_world_to_navigation_orientation.0.data'

path_navigation_position_file = path + 'pose_world_to_navigation_position.0.data'

#######################################
esa_arl_dem_file = '/home/javi/exoter/development/esa_terrain_lab/DEMclean.ply'
#######################################
path_gpy_gaussian_process_model_file = '/home/javi/exoter/dev/bundles/exoter/data/gaussian_processes/SparseGP_RBF_xyz_velocities_train_at_500ms_normalized.data'
#######################################

import sys
sys.path.insert(0, './src/core')
sys.path.insert(0, './src/gaussian_process/benchmarks')
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

from methods import GP_RBF, SVIGP_RBF, SparseGP_RBF, SparseGP_RBF_NL, GP_MAT32, SparseGP_MAT32, GP_MAT52, SparseGP_MAT52

##########################################################################
# PLOTTING FUNCTION
##########################################################################
def arl_dem_figure(fig_num, dem_file, trajectory, pred_mean, kf_trajectory, frames_trajectory, odo_trajectory = None, color_bar='Reds'):

    ########################
    # Load Terrain DEM
    ########################
    plydata = PlyData.read(open(dem_file))

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

    # Display the DEM
    plt.rc('text', usetex=False)# activate latex text rendering
    CS = plt.contour(dem_xi, dem_yi, dem_zi, 15, linewidths=0.5, colors='k')
    CS = plt.contourf(dem_xi, dem_yi, dem_zi, 15, cmap=plt.cm.gray, vmax=abs(dem_zi).max(), vmin=-abs(dem_zi).max())

    # plot data points.
    plt.xlim(min(dem_px), max(dem_xi))
    plt.ylim(min(dem_py), max(dem_yi))

    # Display Ground Truth trajectory
    from numpy import linalg as la
    x = trajectory[:,0]
    y = trajectory[:,1]
    sd = la.norm(pred_mean, axis=1)
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    from matplotlib.collections import LineCollection
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.colors import LinearSegmentedColormap as lscm

    cmap = plt.get_cmap(color_bar)

    #cmap = lscm.from_list('temp', colors)
    #norm = plt.Normalize(0.00, 0.0634491701615)
    norm = plt.Normalize(min(sd), max(sd))
    lc = LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(sd)
    lc.set_linewidth(20)
    lc.set_alpha(0.8)
    plt.gca().add_collection(lc)


    #color bar of the covarianve
    #cbaxes = fig.add_axes([0.8, 0.1, 0.03, 0.8]) 
    h_cbar = plt.colorbar(lc)#, orientation='horizontal')
    h_cbar.ax.set_ylabel(r' gp odometry residual [$m/s$]')

    # Color bar of the dem
    cbar = plt.colorbar()  # draw colorbar
    cbar.ax.set_ylabel(r' terrain elevation[$m$]')

    # Plot the key frames
    fr_x = frames_trajectory[:,0]
    fr_y = frames_trajectory[:,1]
    ax.plot(fr_x, fr_y, marker='s', linestyle='-', lw=2, alpha=0.3, color=[0.0, 0.3, 1.0])

    # Plot all the image frames
    kf_x = kf_trajectory[:,0]
    kf_y = kf_trajectory[:,1]
    ax.scatter(kf_x, kf_y, facecolor=[0.0,1.0,0.0], edgecolor='b', s=150, alpha=1.0)

    # Pure odometry trajectory
    if odo_trajectory is not None:
        odo_x = odo_trajectory[:,0]
        odo_y = odo_trajectory[:,1]
        ax.plot(odo_x, odo_y, linestyle='-.', lw=2, alpha=1.0, color=[0, 0, 0])

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

    ax.annotate(r'ExoTeR', xy=(x[0], y[0]), xycoords='data',
                            xytext=(-20, 30), textcoords='offset points', fontsize=12,
                            #arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0)
                            )

    ax.add_artist(ab)

    plt.xlabel(r'X [$m$]', fontsize=15, fontweight='bold')
    plt.ylabel(r'Y [$m$]', fontsize=15, fontweight='bold')
    #plt.axis('equal')
    plt.grid(True)
    plt.show(block=False)

##########################################################################
# READ THE VALUES IN PANDAS
##########################################################################

#ExoTeR Odometry
odometry = pandas.read_csv(path_odometry_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)


#Reference Position
reference = pandas.read_csv(path_reference_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#ORB_SLAM2 Position
orb_slam2 = pandas.read_csv(path_orb_slam2_position_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#orb_slam2 = orb_slam2[['x', 'y', 'z']].copy()

# KeyFrames Trajectory
keyframes = pandas.read_csv(path_keyframes_trajectory_file, sep=" ", parse_dates=False,
    names=['x', 'y', 'z', 'heading'], header=None)

# Images Frames Trajectory
imageframes = pandas.read_csv(path_allframes_trajectory_file, sep=" ", parse_dates=False,
    names=['x', 'y', 'z', 'heading'], header=None)

#World to Navigation Pose
navigation_orient = data.QuaternionData()
navigation_orient.readData(path_navigation_orientation_file, cov=False)
navigation_position = data.ThreeData()
navigation_position.readData(path_navigation_position_file, cov=False)

##########################################################################
# CREATE THE GP DATASET
##########################################################################

###################
## PREDICTION    ##
###################
path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/'
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
reference_velocity = pandas.read_csv(pose_ref_velocity_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

# Odometry Robot Velocity
odometry_velocity = pandas.read_csv(pose_odo_velocity_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

# IMU orientation (in quaternion form)
data_orient = data.QuaternionData()
data_orient.readData(pose_imu_orientation_file, cov=True)
time = [dateparse(x*1e06) for x in data_orient.atime]
euler = np.column_stack((data_orient.getEuler(2), data_orient.getEuler(1), data_orient.getEuler(0)))
imu_orient = pandas.DataFrame(data=euler, columns=['x', 'y', 'z'], index=time)

# IMU acceleration
imu_acc = pandas.read_csv(pose_imu_acceleration_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z'], header=None)

# IMU Angular Velocity
imu_gyro = pandas.read_csv(pose_imu_angular_velocity_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z'], header=None)

# Robot Joints Position and Speed
names = ["time", "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"]

joints_position = pandas.read_csv(joints_position_file, sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time', names=names, header=None)

joints_speed = pandas.read_csv(joints_speed_file, sep=" ", parse_dates=True,
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
reference_velocity = reference_velocity.resample(resampling_time)
odometry_velocity = odometry_velocity.resample(resampling_time)
imu_orient = imu_orient.resample(resampling_time)
imu_acc = imu_acc.resample(resampling_time)
imu_gyro = imu_gyro.resample(resampling_time)
joints_position = joints_position.resample(resampling_time)
joints_speed = joints_speed.resample(resampling_time)

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
reference = reference.resample(resampling_time)
odometry = odometry.resample(resampling_time)
orb_slam2 = orb_slam2.resample(resampling_time, fill_method='pad')

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

