#!/usr/bin/env python

#######################################
path_odometry_file = '/home/jhidalgocarrio/exoter/development/post-process_data/20140723_pink_odometry/20140723-1826/pose_odo_angular_velocity.0.data'

path_reference_file = '/home/jhidalgocarrio/exoter/development/post-process_data/20140723_pink_odometry/20140723-1826/pose_ref_angular_velocity.0.data'

path_imu_file = '/home/jhidalgocarrio/exoter/development/post-process_data/20140723_pink_odometry/20140723-1826/stim300_gyro.0.data'
#######################################


import sys
sys.path.insert(0, './src/core')
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov

# Odometry angular velocity
odometry = data.ThreeData()
odometry.readData(path_odometry_file, cov=True)
odometry.eigenValues()

#Vicon Angular Velocity
reference = data.ThreeData()
reference.readData(path_reference_file, cov=True)
reference.eigenValues()

#IMU Angular Velocity
imu = data.ThreeData()
imu.readData(path_imu_file, cov=False)
imu.eigenValues()




#Velocity comparison
plt.figure(1)
values = odometry.getAxis(0)
plt.plot(odometry.time, values,
        marker='.', label="Odometry Angular Velocity X-axis", color=[1,0,0], lw=2)
plt.plot(odometry.time, odometry.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry.time, odometry.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)
values=reference.getAxis(0)
plt.plot(reference.time, values,
        marker='D', label="Ground Truth Angular Velocity X-axis", color=[0,0.5,0.5], alpha=0.5, lw=2)

values=imu.getAxis(0)
plt.plot(imu.time, values,
        marker='.', label="IMU Angular Velocity X-axis", color=[1,0,0.4], lw=2)

plt.ylabel(r'Velocity [$rad/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plt.figure(2)
values = odometry.getAxis(1)
plt.plot(odometry.time, values,
        marker='.', label="Odometry Angular Velocity Y-axis", color=[0,1,0], lw=2)
plt.plot(odometry.time, odometry.getStdMax(1, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry.time, odometry.getStdMin(1, 3) , color=[0,0,0], linestyle='--', lw=2)
values=vicon.getAxis(1)
plt.plot(vicon.time, values,
        marker='D', label="Ground Truth Angular Velocity Y-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)

values=imu.getAxis(1)
plt.plot(imu.time, values,
        marker='.', label="IMU Angular Velocity Y-axis", color=[0,1,0.4], lw=2)

plt.ylabel(r'Velocity [$rad/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


plt.figure(3)
values = odometry.getAxis(2)
plt.plot(odometry.time, values,
        marker='.', label="Odometry Angular Velocity Z-axis", color=[0,0,1], lw=2)
plt.plot(odometry.time, odometry.getStdMax(2, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry.time, odometry.getStdMin(2, 3) , color=[0,0,0], linestyle='--', lw=2)
values=vicon.getAxis(2)
plt.plot(vicon.time, values,
        marker='D', label="Ground Truth Angular Velocity Z-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)

values=imu.getAxis(2)
plt.plot(imu.time, values,
        marker='.', label="IMU Angular Velocity Z-axis", color=[0,0.4,1], lw=2)


plt.ylabel(r'Velocity [$rad/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

