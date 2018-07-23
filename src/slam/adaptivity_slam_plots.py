#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2017-02-24 15:17:57

#######################################
path='~/npi/data/20141024_planetary_lab/20141027-2034_orb_slam2_quadratic_adaptivity_five/'
#######################################

path_task_info_adaptive_slam_file = path + 'task_info_adaptive_slam.0.data'
#######################################

import sys
import os
import scipy
from pylab import *
import numpy as np
from numpy import linalg as la
import matplotlib.pyplot as plt

import pandas as pandas
import datetime
matplotlib.style.use('classic') #in matplotlib >= 1.5.1
#pandas.set_option('display.mpl_style', 'default') # Make the graphs a bit prettier
def dateparse (time_in_microsecs):
    return datetime.datetime.fromtimestamp(float(time_in_microsecs) * 1e-06)
##########################################################################
# READ THE VALUES IN PANDAS
##########################################################################
#ExoTeR Odometry
info = pandas.read_csv(os.path.expanduser(path_task_info_adaptive_slam_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'relocalization', 'loops', 'icounts', 'desired_fps',
        'actual_fps', 'inliers_matches_ratio_th', 'map_matches_ratio_th',
        'inliers_matches_th', 'map_matches_ratio_cu',
        'inliers_matches_cu', 'frame_gp_residual', 'kf_gp_residual',
        'kf_gp_threshold', 'distance_traversed'], header=None)

#################
## RE-SAMPLE   ##
#################
resampling_time = '1s'
info = info.resample(resampling_time).mean()

#################
###### PLOT #####
#################
import matplotlib.ticker as ticker

matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
fig, ax1 = plt.subplots()

ax1.plot(info.index.to_datetime(), info.inliers_matches_th,
        linestyle='--', lw=2, alpha=1.0, color=[0.0, 0, 0.0])

ax1.set_ylabel(r'Inliers Matches[$\#$]', fontsize=25, fontweight='bold', color='k')
ax1.tick_params('y', colors='k')

ax2 = ax1.twinx()
ax2.plot(info.index.to_datetime(), info.inliers_matches_ratio_th,
        linestyle='--', lw=2, alpha=1.0, color=[1.0, 0, 0.0])

ax2.set_ylabel(r'Inliers Ratio[$1 - 0$]', fontsize=25, fontweight='bold', color='r')
ax2.tick_params('y', colors='r')

#ticklabels = [item.strftime('%H:%M:%S') for item in info.index]
#ax1.xaxis.set_major_formatter(ticker.FixedFormatter(ticklabels))

ax1.set_xlabel(r'Time [$s$]', fontsize=25, fontweight='bold')
ax1.tick_params('x', colors='k')
plt.grid(True)
plt.legend(loc=1, prop={'size':15})
plt.show(block=True)


#################
###### PLOT #####
#################
matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
fig, ax = plt.subplots()

x = info.index.to_datetime()
y = info.inliers_matches_ratio_th
ax.plot(x, y, linestyle='--', lw=2, alpha=1.0, color=[1.0, 0, 0.0])

d = scipy.zeros(len(x))
ax.fill_between(x, y, 0, color='blue')

ax.set_ylabel(r'Inliers Ratio[$1 - 0$]', fontsize=25, fontweight='bold', color='k')
ax.tick_params('y', colors='k')

ax.set_xlabel(r'Time [$s$]', fontsize=25, fontweight='bold')
ax.tick_params('x', colors='k')
plt.grid(True)
plt.legend(loc=1, prop={'size':15})
plt.show(block=True)

