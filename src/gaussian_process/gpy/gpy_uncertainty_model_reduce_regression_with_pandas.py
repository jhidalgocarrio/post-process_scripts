#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2016-08-25 19:20:08

#######################

#path = '~/npi/data/20141024_gaussian_processes/merged_bis/'
path = '~/npi/data/20140911_gaussian_processes/merged/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

pose_ref_velocity_file =  path + 'delta_pose_ref_velocity.0.data'

pose_odo_velocity_file =  path + 'delta_pose_odo_velocity.0.data'

pose_odo_orientation_file =  path + 'pose_odo_orientation.0.data'

pose_imu_orientation_file =  path + 'pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data'
#######################################

import sys
sys.path.insert(0, './src/core')
sys.path.insert(0, './src/gaussian_process/benchmarks')
import numpy as np
from pylab import *
from matplotlib import pyplot as plt
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov
import joints as js
import GPy
from methods import GP_RBF, SVIGP_RBF, SparseGP_RBF, SparseGP_RBF_NL, GP_MAT32, SparseGP_MAT32, GP_MAT52, SparseGP_MAT52


import pandas as pandas
import datetime
matplotlib.style.use('ggplot')# in matplotlib >= 1.5.1
#pandas.set_option('display.mpl_style', 'default') # Make the graphs a bit prettier
def dateparse (time_in_microsecs):
    return datetime.datetime.fromtimestamp(float(time_in_microsecs) * 1e-06)

def crosscorr(datax, datay, lag=0):
    """ Lag-N cross correlation. 
    Parameters
    ----------
    lag : int, default 0
    datax, datay : pandas.Series objects of equal length

    Returns
    ----------
    crosscorr : float
    """
    return datax.corr(datay.shift(lag))
##########################################################################
# READ THE VALUES IN PANDAS
##########################################################################
import os

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

print(odometry_velocity['x'].sub(odometry_velocity['x'].shift(), fill_value = 0))

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

#Compute the error
odometry_velocity['error_x'] = pandas.Series (fabs(odometry_velocity.x - reference_velocity.x))
odometry_velocity['error_y'] = pandas.Series (fabs(odometry_velocity.y - reference_velocity.y))
odometry_velocity['error_z'] = pandas.Series (fabs(odometry_velocity.z - reference_velocity.z))

#Compute the error
reference_velocity['error_x'] = pandas.Series (fabs(reference_velocity.x - odometry_velocity.x))
reference_velocity['error_y'] = pandas.Series (fabs(reference_velocity.y - odometry_velocity.y))
reference_velocity['error_z'] = pandas.Series (fabs(reference_velocity.z - odometry_velocity.z))

##########################################################################
# PLOT
##########################################################################
matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
plt.rc('text', usetex=False)# activate latex text rendering
fig, ax = plt.subplots()

ax.plot(reference_velocity.index.to_datetime(), reference_velocity.x,
        marker='o', linestyle='-', lw=2,
        color=[1.0, 0, 0],
        label='Reference velocity')

ax.plot(odometry_velocity.index.to_datetime(), odometry_velocity.x,
        marker='x', linestyle='-', lw=2,
        color=[0, 0, 1.0],
        label='Odometry velocity')

ax.plot(odometry_velocity.index.to_datetime(), odometry_velocity.error_x,
        marker='x', linestyle='-', lw=2,
        color=[0, 0, 0],
        label='Error')

ax.set_ylabel(r'Velocity [$m/s$]')

ax2 = ax.twinx()
ax2.plot(imu_orient.index.to_datetime(), imu_orient.x,
        marker='.', linestyle='--', lw=2,
        color=[0, 1.0, 1.0],
        label='Roll')
ax2.set_ylabel(r'Attitude[$rad$]')

lines = ax.get_lines() + ax2.get_lines()
ax.legend(lines, [line.get_label() for line in lines], loc='upper center')

plt.grid(True)
plt.show(block=True)

##########################################################################
# ELIMINATE NULL VALUES
##########################################################################
training_mask = pandas.notnull(reference_velocity.error_x) & pandas.notnull(reference_velocity.error_y) & pandas.notnull(reference_velocity.error_z)
reference_velocity = reference_velocity[training_mask]

training_mask = pandas.notnull(odometry_velocity.error_x) & pandas.notnull(odometry_velocity.error_y) & pandas.notnull(odometry_velocity.error_z)
odometry_velocity = odometry_velocity[training_mask]
imu_orient = imu_orient[training_mask]
imu_acc = imu_acc[training_mask]
imu_gyro = imu_gyro[training_mask]
joints_position = joints_position[training_mask]
joints_speed = joints_speed[training_mask]

##########################################################################
# GAUSSIAN PROCESS X INPUT VECTOR
##########################################################################
X_orientation = np.column_stack((imu_orient.x.as_matrix(),
                imu_orient.y.as_matrix()))

X_joints_position = np.column_stack((
                joints_position.left_passive.as_matrix(),
                joints_position.right_passive.as_matrix(),
                joints_position.rear_passive.as_matrix(),
                joints_position.fl_steer.as_matrix(),
                joints_position.fr_steer.as_matrix(),
                joints_position.rl_steer.as_matrix(),
                joints_position.rr_steer.as_matrix(),
                ))

X_joints_speed = np.column_stack((
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

X_acc =  np.column_stack((
                imu_acc.x.as_matrix(),
                imu_acc.y.as_matrix(),
                imu_acc.z.as_matrix()
                ))

X_gyro =  np.column_stack((
                imu_gyro.x.as_matrix(),
                imu_gyro.y.as_matrix(),
                imu_gyro.z.as_matrix()
                ))

X = np.column_stack((X_orientation, X_joints_position, X_joints_speed, X_acc,
    X_gyro))

##########################################################################
# GAUSSIAN PROCESS Y OUTPUT VECTOR
##########################################################################
Y = np.column_stack((odometry_velocity.error_x.as_matrix(),
                odometry_velocity.error_y.as_matrix(),
                odometry_velocity.error_z.as_matrix()))
#Y = np.row_stack((odometry_velocity.error_x.as_matrix()))

#########################
## GAUSSIAN PROCESS    ##
#########################

# KERNEL
ker_rbf = GPy.kern.RBF(input_dim = X.shape[1], ARD=True) # with ARD one lengthscale parameter per dimension
ker =  ker_rbf

# GP MODEL
m = GPy.models.GPRegression(X, Y, kernel=ker)
print m
m.gradient # indication on understanding the (possibly hard) optimization process.

# OPTIMIZATION
m.optimize('bfgs', messages=True, max_f_eval=1000)
print m

###################
m.plot_f() #Show the predictive values of the GP.

fig, ax = plt.subplots()
for i in range(6):
    m.plot(fignum=1,fixed_inputs=[(1, i)],ax=ax,legend=i==0)
plt.xlabel('years')
plt.ylabel('time/s')

plt.errorbar(X[:,0],Y[:,0],fmt=None,ecolor='r',zorder=1)
plt.grid()
plt.plot(X[:,0],Y[:,0],'kx',mew=1.5)
###################

###################
## PREDICTION    ##
###################
#path = '~/npi/data/20141024_planetary_lab/20141027-2034/'
path = '~/npi/data/20140911_decos_field/20140911-1805/'
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

# Equalize the length of data
reference_velocity = reference_velocity[0:test_mask.shape[0]]
odometry_velocity = odometry_velocity[0:test_mask.shape[0]]
imu_orient = imu_orient[0:test_mask.shape[0]]
imu_acc = imu_acc[0:test_mask.shape[0]]
imu_gyro = imu_gyro[0:test_mask.shape[0]]
joints_position = joints_position[0:test_mask.shape[0]]
joints_speed = joints_speed[0:test_mask.shape[0]]

# Sync index with odometry
reference_velocity.index = test_mask.index
odometry_velocity.index = test_mask.index
imu_orient.index = test_mask.index
imu_acc.index = test_mask.index
imu_gyro.index = test_mask.index
joints_position.index = test_mask.index
joints_speed.index = test_mask.index

# Apply the mask
reference_velocity = reference_velocity[test_mask]
odometry_velocity = odometry_velocity[test_mask]
imu_orient = imu_orient[test_mask]
imu_acc = imu_acc[test_mask]
imu_gyro = imu_gyro[test_mask]
joints_position = joints_position[test_mask]
joints_speed = joints_speed[test_mask]

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

##########################
# FILTER THE TRUTH ERROR
##########################
from scipy.signal import butter, lfilter, freqz

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

order = 6 #filter order
fs = 1.0 # sample rate, Hz
cutoff = 0.05  # desired cutoff frequency of the filter, Hz

# Get the filter coefficients so we can check its frequency response.
b, a = butter_lowpass(cutoff, fs, order)

# Plot the frequency response.
w, h = freqz(b, a, worN=8000)
plt.subplot(1, 1, 1)
plt.plot(0.5*fs*w/np.pi, np.abs(h), 'b')
plt.plot(cutoff, 0.5*np.sqrt(2), 'ko')
plt.axvline(cutoff, color='k')
plt.xlim(0, 0.5*fs)
plt.title("Lowpass Filter Frequency Response")
plt.xlabel('Frequency [Hz]')
plt.grid()
plt.show(block=True)

error = butter_lowpass_filter(odometry_velocity.error_x, cutoff, fs, order)

##########################
matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
fig = plt.figure(2)
ax = fig.add_subplot(111)

ax.plot(reference_velocity.index.to_datetime(), reference_velocity.x,
        marker='o', linestyle='-', lw=2,
        color=[1.0, 0, 0],
        label='Reference velocity')

ax.scatter(reference_velocity.index.to_datetime(), reference_velocity.x,
        marker='D', color=[1.0,0.0,0.0], s=80)

ax.plot(odometry_velocity.index.to_datetime(), odometry_velocity.x,
        marker='x', linestyle='-', lw=2,
        color=[0, 0, 1.0],
        label='Odometry velocity')

ax.plot(odometry_velocity.index.to_datetime(), odometry_velocity.error_x,
        marker='x', linestyle='-', lw=2,
        color=[0, 0, 0],
        label='Error')

[prediction_mean, prediction_var] = m.predict(Xp)

ax.plot(odometry_velocity.index.to_datetime(), prediction_mean[:,0],
        marker='x', linestyle='-', lw=2,
        color=[1.0, 1.0, 0],
        label='Prediction')

sigma = prediction_mean[:,0] + 3.0 * prediction_var[:,0]
ax.plot(odometry_velocity.index.to_datetime(), sigma,
        marker='', linestyle='--', lw=2,
        color=[1.0, 0.7, 0.0])

sigma = prediction_mean[:,0] - 3.0 * prediction_var[:,0]
ax.plot(odometry_velocity.index.to_datetime(), sigma,
        marker='', linestyle='--', lw=2,
        color=[1.0, 0.7, 0.0])

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':15})
plt.show(block=True)

####################################################################################################################################################################################
# SAVE WORKSPACE
####################################################################################################################################################################################

#data.save_object(m, r'./data/gaussian_processes/gpy_uncertainty_model_xyz_velocities_1s_sampling_time.data')
#otro = data.open_object('./data/gpy_model.out')


