#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2016-08-25 19:20:08

#######################

#path = '/home/javi/exoter/development/data/20141023_pink_test/20141023-2011/'
path = '/home/javi/exoter/development/data/20140000_gaussian_processes/merged_bis/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

pose_ref_velocity_file =  path + 'delta_pose_ref_velocity.0.data'

pose_odo_velocity_file =  path + 'delta_pose_odo_velocity.0.data'

pose_odo_orientation_file =  path + 'pose_odo_orientation.0.data'

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data'
#######################################

import sys
sys.path.insert(0, './src/core')
import numpy as np
from pylab import *
from matplotlib import pyplot as plt
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov
import joints as js
import GPy


import pandas as pandas
import datetime
pandas.set_option('display.mpl_style', 'default') # Make the graphs a bit prettier
def dateparse (time_in_microsecs):
    return datetime.datetime.fromtimestamp(float(time_in_microsecs * 1e-06))

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

reference_velocity = pandas.read_csv(pose_ref_velocity_file,sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

odometry_velocity = pandas.read_csv(pose_odo_velocity_file,sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

reference_velocity = reference_velocity.drop(reference_velocity[fabs(reference_velocity.x) > 10].index)
odometry_velocity = odometry_velocity.drop(odometry_velocity[fabs(odometry_velocity.x) > 10].index)

print(odometry_velocity['x'].sub(odometry_velocity['x'].shift(), fill_value = 0))




################
## RE-SAMPLE   ##
################
reference_velocity_sum = reference_velocity.resample('1s')
odometry_velocity_sum = odometry_velocity.resample('1s')

#Compute the error
reference_velocity['error'] = pandas.Series (fabs(odometry_velocity.x - reference_velocity.x))

