#!/usr/bin/env python

#######################################
path_odometry_file = '/home/javi/exoter/development/data/20141024_planetary_lab/20141024-2202/pose_odo_velocity.0.data'

#path_skid_file = '/home/javi/exoter/development/data/20141024_planetary_lab/20141024-2202/pose_skid_position.0.data'

path_reference_file = '/home/javi/exoter/development/data/20141024_planetary_lab/20141024-2202/pose_ref_velocity.0.data'

joints_effort_file = '/home/javi/exoter/development/data/20141024_planetary_lab/20141024-2202/joints_effort.0.data'
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


#ExoTeR Odometry
odometry = data.ThreeData()
odometry.readData(path_odometry_file, cov=True)
odometry.eigenValues()

#Skid Odometry
#skid = data.ThreeData()
#skid.readData(path_skid_file, cov=True)
#skid.eigenValues()


#Vicon Pose
reference = data.ThreeData()
reference.readData(path_reference_file, cov=True)
reference.eigenValues()

#Velocity comparison
plt.figure(1)
values = odometry.getAxis(0)
plt.plot(odometry.time, values,
        marker='.', label="Odometry Velocity X-axis", color=[1,0,0], lw=2)
plt.plot(odometry.time, odometry.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry.time, odometry.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)
values=vicon.getAxis(0)
plt.plot(vicon.time, values,
        marker='D', label="Ground Truth X-axis", color=[0,0.5,0.5], alpha=0.5, lw=2)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plt.figure(2)
values = odometry.getAxis(1)
plt.plot(odometry.time, values,
        marker='.', label="Odometry Velocity Y-axis", color=[1,0,0], lw=2)
plt.plot(odometry.time, odometry.getStdMax(1, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry.time, odometry.getStdMin(1, 3) , color=[0,0,0], linestyle='--', lw=2)
values=vicon.getAxis(1)
plt.plot(vicon.time, values,
        marker='D', label="Ground Truth Y-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plt.figure(1)
values = odometry.getAxis(2)
plt.plot(odometry.time, values,
        marker='.', label="Odometry Velocity Z-axis", color=[1,0,0], lw=2)
plt.plot(odometry.time, odometry.getStdMax(2, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry.time, odometry.getStdMin(2, 3) , color=[0,0,0], linestyle='--', lw=2)
values=vicon.getAxis(2)
plt.plot(vicon.time, values,
        marker='D', label="Ground Truth Z-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


