#!/usr/bin/env python

path = '/home/javi/exoter/development/data/20140422_stim300_vs_vicon/20140429-1429/'

##################################
pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data' # IMU is wrt to body frame

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data' # IMU is wrt to body frame

pose_imu_inclinometer_file =  path + 'pose_imu_inclinometer.0.data' # IMU is wrt to body frame
##################################

import sys
sys.path.insert(0, './src/core')
import csv, scipy
from pylab import *
import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
import quaternion as quat
import datadisplay as data
sys.path.insert(0, './src/attitude')
import quater_ikf

# Angular velocity
gyro = data.ThreeData()
gyro.readData(pose_imu_angular_velocity_file, cov=False)

# Acceleration
acc = data.ThreeData()
acc.readData(pose_imu_acceleration_file, cov=False)

# Inclinometers
inc = data.ThreeData()
inc.readData(pose_imu_inclinometer_file, cov=False)

# Sensor information
gyro_bandwidth = 33.0
gyro_delta_t = 1.0/gyro_bandwidth

acc_bandwidth = 33.0
acc_delta_t = 1.0/acc_bandwidth

inc_bandwidth = 16.0
inc_delta_t = 1.0/inc_bandwidth

# Noise Parameters
acc_randomwalk = np.array([0.0005420144, 0.0005131682, 0.0004908665])
gyro_randomwalk = np.array([3.320343e-05, 4.455001e-05, 4.060973e-05])
inc_randomwalk = np.array([0.005019287, 0.005019287, 0.005019287])

acc_biasinstability = np.array([0.0004368486, 0.0003441604, 0.0003097561])
gyro_biasinstability = np.array([7.05e-06, 4.82e-06, 6.36e-06])
inc_biasinstability = np.array([0.008292219, 0.008160451, 0.00846485])

# From the Covariance matrices
Ra = np.matlib.eye(3, dtype=double)
Ra[0,0] = pow(acc_randomwalk[0]/sqrt(acc_delta_t), 2)
Ra[1,1] = pow(acc_randomwalk[1]/sqrt(acc_delta_t), 2)
Ra[2,2] = pow(acc_randomwalk[2]/sqrt(acc_delta_t), 2)


Rg = np.matlib.eye(3, dtype=double)
Rg[0,0] = pow(gyro_randomwalk[0]/sqrt(gyro_delta_t), 2)
Rg[1,1] = pow(gyro_randomwalk[1]/sqrt(gyro_delta_t), 2)
Rg[2,2] = pow(gyro_randomwalk[2]/sqrt(gyro_delta_t), 2)

Ri = np.matlib.eye(3, dtype=double)
Ri[0,0] = pow(inc_randomwalk[0]/sqrt(inc_delta_t), 2)
Ri[1,1] = pow(inc_randomwalk[1]/sqrt(inc_delta_t), 2)
Ri[2,2] = pow(inc_randomwalk[2]/sqrt(inc_delta_t), 2)

Qba = np.matlib.eye(3, dtype=double)
Qba[0,0] = pow(acc_biasinstability[0], 2)
Qba[1,1] = pow(acc_biasinstability[1], 2)
Qba[2,2] = pow(acc_biasinstability[2], 2)

Qbg = np.matlib.eye(3, dtype=double)
Qbg[0,0] = pow(gyro_biasinstability[0], 2)
Qbg[1,1] = pow(gyro_biasinstability[1], 2)
Qbg[2,2] = pow(gyro_biasinstability[2], 2)

Qbi = np.matlib.eye(3, dtype=double)
Qbi[0,0] = pow(inc_biasinstability[0], 2)
Qbi[1,1] = pow(inc_biasinstability[1], 2)
Qbi[2,2] = pow(inc_biasinstability[2], 2)

# Initial covariance
P0 = np.matlib.eye(12, dtype=double)
P0[0:3, 0:3] = np.matlib.eye(3, dtype=double) * 0.001
P0[3:6, 3:6] = np.matlib.eye(3, dtype=double) * 0.01
P0[6:9, 6:9] = np.matlib.eye(3, dtype=double) * 0.01
P0[9:12, 9:12] = np.matlib.eye(3, dtype=double) * 0.01

# Adaptive parameters
acc_m1 = 5
acc_m2 = 5
acc_gamma = 0.001

inc_m1 = 10
inc_m2 = 2
inc_gamma = 0.003

# Vector to matrices
ya = np.asmatrix(acc.data.transpose())
yg = np.asmatrix(gyro.data.transpose())
yi = np.asmatrix(inc.data.transpose())
dip_angle=None
ym = None
tt = np.asmatrix(gyro.t).transpose()

quater_ikf_orient, quater_ikf_euler, bahat, bghat, Qa, Qi = quater_ikf.filter (P0 = P0, ya = None, yg = yg, ym = ym, yi = None, tt=tt,
                                                Ra=Ra, Rg=Rg, Ri=Ri,
                                                Qba=Qba, Qbg=Qbg, Qbi=Qbi, dip_angle = dip_angle,
                                                acc_m1 = acc_m1, acc_m2 = acc_m2, acc_gamma = acc_gamma,
                                                inc_m1 = inc_m1, inc_m2 = inc_m2, inc_gamma = inc_gamma)

##################################
# PLOT AND COMPARE THE RESULTS
##################################
pose_ref_orient_file = path + 'pose_ref_orientation.0.data'

pose_imu_orient_file = path + 'pose_imu_orientation.0.data'
##################################

# Read the reference orientation information
reference_orient = data.QuaternionData()
reference_orient.readData(pose_ref_orient_file, cov=True)

# Read the imu orientation information
imu_orient = data.QuaternionData()
imu_orient.readData(pose_imu_orient_file, cov=True)

################################
### COMPUTE COV EIGENVALUES  ###
################################
imu_orient.covSymmetry()
imu_orient.eigenValues()


#Plotting Orientation values
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.rc('text', usetex=False)# activate latex text rendering

# IMU Orientation
time = imu_orient.t
euler = []
euler.append(imu_orient.getEuler(2))# Roll
euler.append(imu_orient.getEuler(1))# Pitch
euler.append(imu_orient.getEuler(0))# Yaw

#euler[2] = euler[2] - euler[2][0] # set yaw staring at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

axis = 1
if axis == 0:
    label_text = "IMU Roll"
    color_value = [1.0,0,0]
elif axis  == 1:
    label_text = "IMU Pitch"
    color_value = [0.0,1.0,0]
else:
    label_text = "IMU Yaw"
    color_value = [0.0,0.0,1.0]

sigma = imu_orient.getStd(axis=axis, levelconf = 3)
ax.plot(time, euler[axis], marker='.', label=label_text, color=color_value, alpha=0.5, lw=2)
ax.fill(np.concatenate([time, time[::-1]]),
        np.concatenate([euler[axis] - sigma,
                       (euler[axis] + sigma)[::-1]]),
        alpha=.5, fc='b', ec='None', label='95% confidence interval')

ax.plot(time, (euler[axis] - sigma), color="black", alpha=1.0, lw=1.0)
ax.plot(time, (euler[axis] + sigma), color="black", alpha=1.0, lw=1.0)

# Python Filter Values
time = tt
euler=[]
euler = np.array(quater_ikf_euler[:])

#euler[2] = euler[2] - euler[2][0] # set yaw staring at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

if axis == 0:
    label_text = "INT IMU Roll"
    color_value = [0.5,0,0.4]
elif axis  == 1:
    label_text = "INT IMU Pitch"
    color_value = [0.0,0.5,0.4]
else:
    label_text = "INT IMU Yaw"
    color_value = [0.4,0.0,0.5]

ax.plot(time, euler[axis], marker='.', label=label_text, color=color_value, alpha=0.5, lw=2)

# Reference Attitude
time = reference_orient.t
euler = []
euler.append(reference_orient.getEuler(2))# Roll
euler.append(reference_orient.getEuler(1))# Pitch
euler.append(reference_orient.getEuler(0))# Yaw

#euler[2] = euler[2] - euler[2][0] #set yaw starting at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

if axis == 0:
    label_text = "Vicon Roll"
    color_value = [0.7,0.4,0]
elif axis  == 1:
    label_text = "Vicon Pitch"
    color_value = [0.4,0.7,0]
else:
    label_text = "Vicon Yaw"
    color_value = [0,0.4,0.7]

ax.plot(time, euler[axis], marker='.', label=label_text, color=color_value, alpha=0.5, lw=2)

plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Angle [${}^\circ$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

