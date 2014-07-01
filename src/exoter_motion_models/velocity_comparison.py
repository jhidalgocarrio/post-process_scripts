#!/usr/bin/env python

import sys
sys.path.insert(0, './src/core')
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
import datadisplay as data

#ExoTeR Odometry
odometry = data.ThreeData()
#odometry.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140600_pink_odometry_test/20140605-1731/pose_odo_velocity.0.data', cov=True)
odometry.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140600_pink_odometry_test/20140630-1847/pose_odo_velocity.0.data', cov=True)
odometry.eigenValues()

#Vicon Pose
vicon = data.ThreeData()
#vicon.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140600_pink_odometry_test/20140605-1731/pose_ref_velocity.0.data', cov=False)
vicon.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140600_pink_odometry_test/20140630-1847/pose_ref_velocity.0.data', cov=False)
vicon.eigenValues()

#IMU Acc
imu = data.ThreeData()
imu.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140600_pink_odometry_test/20140605-1731/inertial_samples_acc.0.data', cov=False)
imu.eigenValues()


#Velocity comparison
plt.figure(1)
values = odometry.getAxis(0)
plt.plot(odometry.time, values,
        marker='.', label="Odometry Velocity X-axis", color=[1,0,0], lw=2)
plt.plot(odometry.time, odometry.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry.time, odometry.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)
values=vicon.getAxis(0)
plt.plot(vicon.time, values,
        marker='D', label="Ground Truth X-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plt.figure(1)
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



# IMU acceleration integration
velimu = []
velimu.append(imu.getAxis(0))
velimu.append(imu.getAxis(1))
velimu.append(imu.getAxis(2))

velimu[0][:] = [x * mean(imu.delta) for x in velimu[0] ]#convert to increments in velocity
velimu[1][:] = [x * mean(imu.delta) for x in velimu[1] ]#convert to increments in velocity
velimu[2][:] = [x * mean(imu.delta) for x in velimu[2] ]#convert to increments in velocity


plt.figure(1)
plt.plot(imu.time, imu.getAxis(0),
        marker='.', label="IMU Acc X-axis", color=[1,0.3,0], lw=2)
plt.plot(imu.time, np.cumsum(velimu[0]),
        marker='.', label="IMU Velocity X-axis", color=[1,0,0], lw=2)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plt.figure(1)
plt.plot(imu.time, imu.getAxis(1),
        marker='.', label="IMU Acc Y-axis", color=[0.3,1,0], lw=2)
plt.plot(imu.time, np.cumsum(velimu[1]),
        marker='.', label="IMU Velocity Y-axis", color=[0,1,0], lw=2)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

