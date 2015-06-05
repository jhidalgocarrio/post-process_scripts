#!/usr/bin/env python

path = '/home/javi/flatfish/development/data/20150312_dagon_estimator_evaluation_1h/20150312-1715/plots/'

##################################
pose_ikf_orient_file = path + 'orientation_odometry.csv'

pose_ref_orient_file = path + 'orientation_ground_truth.csv'
##################################

import sys
sys.path.insert(0, './src/core')
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
import datadisplay as data

# Read the imu orientation information
ikf_orient = data.QuaternionData()
ikf_orient.readData(pose_ikf_orient_file, cov=True)

# Read the reference orientation information
reference_orient = data.QuaternionData()
reference_orient.readData(pose_ref_orient_file, cov=True)

################################
### COMPUTE COV EIGENVALUES  ###
################################
ikf_orient.covSymmetry()
ikf_orient.eigenValues()

reference_orient.covSymmetry()
reference_orient.eigenValues()

########################
### PLOTTING VALUES  ###
########################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering

time = ikf_orient.atime
time = time - ikf_orient.atime[0] # Time alignment
time[:] = [(x*1000.00)/60.0 for x in time] # Convert time to minutes
euler = []
euler.append(ikf_orient.getEuler(2))# Roll
euler.append(ikf_orient.getEuler(1))# Pitch
euler.append(ikf_orient.getEuler(0))# Yaw

# Convert to degrees
euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

# Reduce number of points
time = time[0::50]
euler[0] = euler[0][0::50]
euler[1] = euler[1][0::50]
euler[2] = euler[2][0::50]

# IKF Filter
axis = 2
if axis == 0:
    label_text = "Roll [Dagon AHRS w/ Allan data]"
    color_value = [1.0,0,0]
elif axis  == 1:
    label_text = "Pitch [Dagon AHRS w/ Allan data]"
    color_value = [0.0,1.0,0]
else:
    label_text = "Yaw [Dagon AHRS w/ Allan data]"
    color_value = [0.0,0.0,1.0]

ax.plot(time, euler[axis], marker='o', linestyle='-', label=label_text, color=color_value, lw=2)
sigma = ikf_orient.getStd(axis=axis, levelconf = 2)
sigma[:] = [x * 180.00/math.pi for x in sigma]#convert to degrees
sigma = sigma[0::50]
ax.fill(np.concatenate([time, time[::-1]]),
        np.concatenate([euler[axis] - sigma,
                       (euler[axis] + sigma)[::-1]]),
        alpha=.5, fc='0.50', ec='None', label='95% confidence interval')

# Ground Truth
time = reference_orient.atime
time = time - ikf_orient.atime[0]# Time alignment
time[:] = [(x*1000.00)/60.0 for x in time] # Convert time to minutes
euler = []
euler.append(reference_orient.getEuler(2))# Roll
euler.append(reference_orient.getEuler(1))# Pitch
euler.append(reference_orient.getEuler(0))# Yaw

# Convert to degrees
euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

# Reduce number of points
time = time[0::5]
euler[0] = euler[0][0::5]
euler[1] = euler[1][0::5]
euler[2] = euler[2][0::5]

if axis == 0:
    label_text = "Roll [ground truth]"
    color_value = [0.2,0.6,0.7]
elif axis  == 1:
    label_text = "Pitch [ground truth]"
    color_value = [0.2,0.6,0.7]
else:
    label_text = "Yaw [ground truth]"
    color_value = [0.2,0.6,0.7]

ax.plot(time, euler[axis], marker='D', linestyle='None', label=label_text, color=color_value, alpha=0.5, lw=2)



plt.xlabel(r'Time [$min$]')
plt.ylabel(r'Angle [${}^\circ$]')
plt.grid(True)
plt.legend(prop={'size':25}, loc=1)
plt.show(block=False)

