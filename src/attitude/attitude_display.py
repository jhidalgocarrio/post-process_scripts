#!/usr/bin/env python

path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141025-0005/'
#path = '/home/javi/exoter/development/data/20140422_stim300_vs_vicon/20140425-1923/'
#path = '/home/javi/exoter/development/data/20150323_ww_dlr/imu_stim300_attitude_test_20150325-1716/'

##################################
pose_odo_orient_file = path + 'pose_odo_orientation.0.data'

pose_ref_orient_file = path + 'pose_ref_orientation.0.data'

pose_imu_orient_file = path + 'pose_imu_orientation.0.data'
##################################

import sys
sys.path.insert(0, './src/core')
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
import datadisplay as data

# Read the odometry orientation information
odometry_orient = data.QuaternionData()
odometry_orient.readData(pose_odo_orient_file, cov=True)

# Read the reference orientation information
reference_orient = data.QuaternionData()
reference_orient.readData(pose_ref_orient_file, cov=True)

# Read the imu orientation information
imu_orient = data.QuaternionData()
imu_orient.readData(pose_imu_orient_file, cov=True)

########################
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference_orient.cov[:,0,0]))
temindex = np.asarray(temindex)

odometry_orient.delete(temindex)
reference_orient.delete(temindex)
imu_orient.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
odometry_orient.covSymmetry()
odometry_orient.eigenValues()

reference_orient.covSymmetry()
reference_orient.eigenValues()

imu_orient.covSymmetry()
imu_orient.eigenValues()


#Plotting Orientation values
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.rc('text', usetex=False)# activate latex text rendering

time = imu_orient.time
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
    label_text = "Roll [filter w/ Allanvar]"
    color_value = [1.0,0,0]
elif axis  == 1:
    label_text = "Pitch [filter w/ Allanvar]"
    color_value = [0.0,1.0,0]
else:
    label_text = "Yaw [filter w/ Allanvar]"
    color_value = [0.0,0.0,1.0]

# IMU Orientation
sigma = imu_orient.getStd(axis=axis, levelconf = 2)
ax.plot(time, euler[axis], marker='.', label=label_text, color=color_value, alpha=0.5, lw=2)
ax.fill(np.concatenate([time, time[::-1]]),
        np.concatenate([euler[axis] - sigma,
                       (euler[axis] + sigma)[::-1]]),
        alpha=.5, fc='b', ec='None', label='95% confidence interval')

ax.plot(time, (euler[axis] - sigma), color="black", alpha=1.0, lw=1.0)
ax.plot(time, (euler[axis] + sigma), color="black", alpha=1.0, lw=1.0)

# Odometry Orientation
time = odometry_orient.time
euler = []
euler.append(odometry_orient.getEuler(2))# Roll
euler.append(odometry_orient.getEuler(1))# Pitch
euler.append(odometry_orient.getEuler(0))# Yaw

#euler[2] = euler[2] - euler[2][0] #set yaw starting at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

# Reduce number of points
time = time[0::20]
euler[0] = euler[0][0::20]
euler[1] = euler[1][0::20]
euler[2] = euler[2][0::20]

if axis == 0:
    label_text = "Roll [filter w/ Allanvar]"
    color_value = [1.0,0.0,0]
elif axis  == 1:
    label_text = "Pitch [filter w/ Allanvar]"
    color_value = [0.0,1.0,0]
else:
    label_text = "Yaw [filter w/ Allanvar]"
    color_value = [0,0.0,1.0]

# Odometry Orientation
ax.plot(time, euler[axis], marker='o', linestyle='-', label=label_text, color=color_value, lw=2)
sigma = odometry_orient.getStd(axis=axis, levelconf = 2)
sigma[:] = [x * 180.00/math.pi for x in sigma]#convert to degrees
sigma = sigma[0::20]
ax.fill(np.concatenate([time, time[::-1]]),
        np.concatenate([euler[axis] - sigma,
                       (euler[axis] + sigma)[::-1]]),
        alpha=.5, fc='0.50', ec='None', label='95% confidence interval')

#ax.plot(time, (euler[axis] - sigma), color="black", alpha=1.0, lw=1.0)
#ax.plot(time, (euler[axis] + sigma), color="black", alpha=1.0, lw=1.0)


# Reference Orientation
time = reference_orient.time
euler = []
euler.append(reference_orient.getEuler(2))# Roll
euler.append(reference_orient.getEuler(1))# Pitch
euler.append(reference_orient.getEuler(0))# Yaw

#Misalignment
alignement_diff = []
alignement_diff.append(odometry_orient.getEuler(2)[0::20][118] - reference_orient.getEuler(2)[0::20][118]) # Roll
alignement_diff.append(odometry_orient.getEuler(1)[0::20][118] - reference_orient.getEuler(1)[0::20][118]) # Roll
alignement_diff.append(odometry_orient.getEuler(0)[0::20][118] - reference_orient.getEuler(0)[0::20][118]) # Roll

euler[0] = euler[0] + alignement_diff[0]
euler[1] = euler[1] + alignement_diff[1]
euler[2] = euler[2] + alignement_diff[2]

# Convert to degrees
euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

# Reduce number of points
time = time[0::20]
euler[0] = euler[0][0::20]
euler[1] = euler[1][0::20]
euler[2] = euler[2][0::20]


if axis == 0:
    label_text = "Roll [ground truth]"
    color_value = [0.2,0.6,0.7]
elif axis  == 1:
    label_text = "Pitch [ground truth]"
    color_value = [0.2,0.6,0.7]
else:
    label_text = "Yaw [ground truth]"
    color_value = [0.2,0.6,0.7]

# Reference Orientation
#sigma = reference_orient.getStd(axis=axis, levelconf = 3)
ax.plot(time[118:len(euler[axis])], euler[axis][118:len(euler[axis])], marker='D', linestyle='None', label=label_text, color=color_value, alpha=0.5, lw=2)
sigma = reference_orient.getStd(axis=axis, levelconf = 2)
sigma[:] = [x * 180.00/math.pi for x in sigma]#convert to degrees
sigma = sigma[0::20]
#ax.fill(np.concatenate([time, time[::-1]]),
#        np.concatenate([euler[axis] - sigma,
#                       (euler[axis] + sigma)[::-1]]),
#        alpha=.5, fc='0.50', ec='None', label='95% confidence interval')

plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Angle [${}^\circ$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

#Plotting Orientation values with variable line width
from matplotlib.collections import LineCollection
fig = plt.figure(2)
ax = fig.add_subplot(111)

# IMU Orientation
time = imu_orient.t
euler = []
euler.append(imu_orient.getEuler(2))# Roll
euler.append(imu_orient.getEuler(1))# Pitch
euler.append(imu_orient.getEuler(0))# Yaw

euler[2] = euler[2] - euler[2][0] # set yaw staring at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

ax.plot(time, euler[0], marker='.', label="IMU Roll", color=[1.0,0,0], alpha=0.5, lw=2)

# Reference Orientation
time = reference_orient.t
euler = []
euler.append(reference_orient.getEuler(2))# Roll
euler.append(reference_orient.getEuler(1))# Pitch
euler.append(reference_orient.getEuler(0))# Yaw

euler[2] = euler[2] - euler[2][0] #set yaw starting at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

#Line width
uncer = np.nan_to_num(np.array(reference_orient.getStd(axis=axis, levelconf=3))).real
lwidths = 0.5 + 300 * uncer[:-1]


points = np.array([time, euler[0]]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
lc = LineCollection(segments, linewidths=lwidths,colors='blue')
ax.add_collection(lc)

plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Angle [${}^\circ$]')

limmin = points[0]
limmax = points[len(points)-1]
ax.set_xlim(limmin[0][0], limmax[0][0])
ax.set_ylim(limmin[0][1]-10, limmax[0][1]+10)

plt.grid(True)
ax.legend(prop={'size':25})
plt.show(block=False)


# Time stamp statistics
deltaimu = []
for i in range(0,len(imu_orient.t)-1):
    #print time[i]
    timu = float(imu_orient.t[i+1]) - float(imu_orient.t[i])
    deltaimu.append(timu)

deltaimu_t = mean(deltaimu)


deltavicon = []
for i in range(0,len(reference_orient.t)-1):
    #print time[i]
    tvicon = float(reference_orient.t[i+1]) - float(reference_orient.t[i])
    deltavicon.append(tvicon)

deltavicon_t = mean(deltavicon)




from matplotlib.collections import LineCollection
x=linspace(0,4*pi,373)
y=cos(x)
y=euler[0][0:373]
temp = np.nan_to_num(np.array(reference_orient.getStd(axis=axis, levelconf=3)))[:-1].real[0:373]
lwidths = 300 * temp[:-1]
points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
lc = LineCollection(segments, linewidths=lwidths,colors='blue')
fig,a = plt.subplots()
a.add_collection(lc)
#a.set_xlim(0,4*pi)
#a.set_ylim(-1.1,1.1)
a.grid(True)
fig.show()
