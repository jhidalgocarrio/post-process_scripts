#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2017-03-28 16:52:14
path = '~/npi/data/20150515_planetary_lab/20150515-1752_odometry_comparison/'

# ################## #
# Planetary Lab Data #
# ################## #
threed_odometry_file = path + 'pose_odo_position.0.data'

skid_odometry_file = path + 'pose_skid_position.0.data'

reference_file =  path + 'pose_ref_position.0.data'
#######################################
delta_reference_file =  path + 'delta_pose_ref_position.0.data'
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
matplotlib.style.use('classic') #in matplotlib >= 1.5.1

def dateparse (time_in_microsecs):
    return datetime.datetime.fromtimestamp(float(time_in_microsecs) * 1e-06)


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

#Vicon Delta Pose
delta_reference = pandas.read_csv(os.path.expanduser(delta_reference_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

##########################################################################
odometry_mask = pandas.notnull(threed_odometry.x) & pandas.notnull(threed_odometry.y) & pandas.notnull(threed_odometry.z)
threed_odometry = threed_odometry[odometry_mask]

skid_mask = pandas.notnull(skid_odometry.x) & pandas.notnull(skid_odometry.y) & pandas.notnull(skid_odometry.z)
skid_odometry = skid_odometry[skid_mask]

ref_mask = pandas.notnull(reference.x) & pandas.notnull(reference.y) & pandas.notnull(reference.z)
reference = reference[ref_mask]

delta_ref_mask = pandas.notnull(delta_reference.x) & pandas.notnull(delta_reference.y) & pandas.notnull(delta_reference.z)
delta_reference = delta_reference[delta_ref_mask]

###########################
##   Distance traveled   ##
###########################
position = np.column_stack((delta_reference.x.values, delta_reference.y.values, delta_reference.z.values))

norm_delta_position = []
norm_delta_position = [np.linalg.norm(x) for x in position ]
norm_delta_position = [x for x in norm_delta_position if str(x) != 'inf']
distance_traveled = np.nansum(norm_delta_position)

#################
## RE-SAMPLE   ##
#################
resampling_time = '500ms'
threed_odometry = threed_odometry.resample(resampling_time).mean()
skid_odometry = skid_odometry.resample(resampling_time).mean()
reference = reference.resample(resampling_time).mean()

# Compute the 3d odometry error
threed_odometry['error_x'] = pandas.Series (fabs(threed_odometry.x - reference.x))
threed_odometry['error_y'] = pandas.Series (fabs(threed_odometry.y - reference.y))
threed_odometry['error_z'] = pandas.Series (fabs(threed_odometry.z - reference.z))
odometry_mask = pandas.notnull(threed_odometry.error_x) & pandas.notnull(threed_odometry.error_y) & pandas.notnull(threed_odometry.error_z)
threed_odometry = threed_odometry[odometry_mask]

# RMSE for Full Odometry
odormse = []
odormse.append(sqrt(((threed_odometry.error_x.values) ** 2).mean()))
odormse.append(sqrt(((threed_odometry.error_y.values) ** 2).mean()))
odormse.append(sqrt(((threed_odometry.error_z.values) ** 2).mean()))

#Final error position for Full Odometry
odofinale = []
odofinale.append(np.absolute(threed_odometry.error_x.values[threed_odometry.shape[0]-1]))
odofinale.append(np.absolute(threed_odometry.error_y.values[threed_odometry.shape[0]-1]))
odofinale.append(np.absolute(threed_odometry.error_z.values[threed_odometry.shape[0]-1]))

# Maximum error for Full Odometry
odomaxe = []
odomaxe.append(threed_odometry.error_x.values.max())
odomaxe.append(threed_odometry.error_y.values.max())
odomaxe.append(threed_odometry.error_z.values.max())

# Median error for Full Odometry
odomediane = []
odomediane.append(np.median(np.absolute(threed_odometry.error_x.values)))
odomediane.append(np.median(np.absolute(threed_odometry.error_y.values)))
odomediane.append(np.median(np.absolute(threed_odometry.error_z.values)))

# Compute the skid odometry error
skid_odometry['error_x'] = pandas.Series (fabs(skid_odometry.x - reference.x))
skid_odometry['error_y'] = pandas.Series (fabs(skid_odometry.y - reference.y))
skid_odometry['error_z'] = pandas.Series (fabs(skid_odometry.z - reference.z))
skid_mask = pandas.notnull(skid_odometry.error_x) & pandas.notnull(skid_odometry.error_y) & pandas.notnull(skid_odometry.error_z)
skid_odometry = skid_odometry[skid_mask]

# RMSE for Skid Odometry
skidrmse = []
skidrmse.append(sqrt(((skid_odometry.error_x.values) ** 2).mean()))
skidrmse.append(sqrt(((skid_odometry.error_y.values) ** 2).mean()))
skidrmse.append(sqrt(((skid_odometry.error_z.values) ** 2).mean()))

#Final error position for Skid Odometry
skidfinale = []
skidfinale.append(np.absolute(skid_odometry.error_x.values[skid_odometry.shape[0]-1]))
skidfinale.append(np.absolute(skid_odometry.error_y.values[skid_odometry.shape[0]-1]))
skidfinale.append(np.absolute(skid_odometry.error_z.values[skid_odometry.shape[0]-1]))

# Maximum error for Skid Odometry
skidmaxe = []
skidmaxe.append(skid_odometry.error_x.values.max())
skidmaxe.append(skid_odometry.error_y.values.max())
skidmaxe.append(skid_odometry.error_z.values.max())

# Median error for Skid Odometry
skidmediane = []
skidmediane.append(np.median(np.absolute(skid_odometry.error_x.values)))
skidmediane.append(np.median(np.absolute(skid_odometry.error_y.values)))
skidmediane.append(np.median(np.absolute(skid_odometry.error_z.values)))

# RMSE for Reference (it shoudl be zero)

from numpy import linalg as la

# Print values
print("3d odometry rmse:" + str(la.norm(odormse)))
print("skid odometry rmse:" + str(la.norm(skidrmse)))

print("3d odometry max error:" + str(la.norm(odomaxe)))
print("skid odometry max error:" + str(la.norm(skidmaxe)))

print("3d odometry median error:" + str(la.norm(odomediane)))
print("skid odometry median error:" + str(la.norm(skidmediane)))

print("3d odometry final error:" + str(la.norm(odofinale)))
print("skid odometry median error:" + str(la.norm(skidfinale)))

print("3d odometry percentage error:" + str((la.norm(odomaxe)*100.0)/distance_traveled))
print("skid odometry percentage error:" + str((la.norm(skidmaxe)*100.0)/distance_traveled))

