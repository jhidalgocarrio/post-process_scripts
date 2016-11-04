#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2016-09-13 17:31:28
from __future__ import print_function
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

from evaluation import RMSE, MAE, MAPE
from tasks import ExoTerOdometryResiduals
from methods import GP_RBF, SVIGP_RBF, SparseGP_RBF, SparseGP_RBF_NL, GP_MAT32, SparseGP_MAT32, GP_MAT52, SparseGP_MAT52
from figures import ExoTerFigures
import numpy as np
import time

import pandas as pandas
import datetime
matplotlib.style.use('ggplot') #in matplotlib >= 1.5.1
#pandas.set_option('display.mpl_style', 'default') # Make the graphs a bit prettier
def dateparse (time_in_microsecs):
    return datetime.datetime.fromtimestamp(float(time_in_microsecs * 1e-06))

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

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
#path_gpy_gaussian_process_model_file = '/home/javi/exoter/dev/bundles/exoter/data/gaussian_processes/SparseGP_RBF_xyz_velocities_train_at_500ms_normalized.data'
#path_gpy_gaussian_process_model_file = '/home/javi/exoter/dev/bundles/exoter/data/gaussian_processes/GP_RBF_xyz_velocities_train_at_1s_normalized.data'
path_gpy_gaussian_process_model_file = '/home/javi/exoter/dev/bundles/exoter/data/gaussian_processes/SparseGP_RBF_xyz_velocities_train_at_1s_normalized.data'
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

Xp_all = np.column_stack((Xp_orientation, Xp_joints_position , Xp_joints_speed,
    Xp_acc, Xp_gyro))

Xp_no_attitude = np.column_stack((Xp_orientation * 0.00, Xp_joints_position , Xp_joints_speed,
    Xp_acc, Xp_gyro))

Xp_no_inertia = np.column_stack((Xp_orientation, Xp_joints_position, Xp_joints_speed,
    Xp_acc * 0.00, Xp_gyro * 0.00))

Xp_no_joints = np.column_stack((Xp_orientation, Xp_joints_position * 0.00, Xp_joints_speed * 0.00,
    Xp_acc, Xp_gyro))

Xp_no_attitude_inertia = np.column_stack((Xp_orientation * 0.00, Xp_joints_position, Xp_joints_speed,
    Xp_acc * 0.00, Xp_gyro * 0.00))

Xp_no_attitude_joints = np.column_stack((Xp_orientation * 0.00, Xp_joints_position * 0.00, Xp_joints_speed * 0.00,
    Xp_acc, Xp_gyro ))

Xp_no_inertia_joints = np.column_stack((Xp_orientation, Xp_joints_position * 0.00, Xp_joints_speed * 0.00,
    Xp_acc * 0.00, Xp_gyro * 0.00))


##########################################################################
# GAUSSIAN PROCESS Y OUTPUT VECTOR
##########################################################################
Yp = np.column_stack((odometry_velocity.error_x.as_matrix(),
        odometry_velocity.error_y.as_matrix(),
        odometry_velocity.error_z.as_matrix()))

##########################################################################
config = {
        'tasks':[ExoTerOdometryResiduals],
        'inputs': [Xp_all, Xp_no_attitude, Xp_no_joints, Xp_no_inertia, Xp_no_attitude_inertia, Xp_no_attitude_joints, Xp_no_inertia_joints],
        'evaluations':[RMSE, MAE, MAPE],
        }


if __name__=='__main__':
    #Load the task
    dataset = config['tasks'][0]()
    print(bcolors.HEADER  + bcolors.BOLD + 'Benchmarking on '+dataset.name + bcolors.ENDC)
    res = dataset.load_data()
    dataset.get_test_data('1s')

    #Load the model
    m = data.open_object(path_gpy_gaussian_process_model_file)
    print (m.model)
    if m.preprocess:
        print(bcolors.BOLD + 'Model: '+ m.name + " is normalized" + bcolors.ENDC)
    else:
        print(bcolors.BOLD + 'Model: '+ m.name + " is unnormalized" + bcolors.ENDC)

    for input_i in range(len(config['inputs'])):
        X_input = np.column_stack((config['inputs'][input_i]))
        test = (np.column_stack((X_input)), Yp)
        print(bcolors.WARNING + 'Selecting '+str(input_i) + " with Xp " + str(test[0].shape) + " and Yp " + str(test[1].shape) + bcolors.ENDC)
        [pred_test_mean, pred_test_var] = m.predict(test[0])
        for ei in range(len(config['evaluations'])):
            evalu = config['evaluations'][ei]()
            eval_test = evalu.evaluate(test[1], pred_test_mean)
            print(bcolors.OKBLUE + 'With evaluation method '+evalu.name + bcolors.ENDC, end=' ')
            print('ERROR Test ['+ bcolors.FAIL + str(eval_test.mean()) + bcolors.ENDC + ']')
        dataset.arl_dem_figure(input_i, m.name+"_inputs_"+str(input_i), pred_test_mean, pred_test_var, '1s', '1s')

