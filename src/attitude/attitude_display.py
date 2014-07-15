#!/usr/bin/env python

import sys
sys.path.insert(0, './src/core')
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
import datadisplay as data

# Read the imu orientation information
imuOrient = data.QuaternionData()
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140422-1505/stim300_filter_33bnw_16bnw_125hz.data', cov=True)
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140422-1539/stim300_filter_33bnw_16bnw_125hz.data', cov=True)
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140423-1923/stim300_filter_33bnw_16bnw_125hz.data', cov=True)
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140423-1942/stim300_filter_33bnw_16bnw_125hz.data', cov=True)
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140425-1923/stim300_filter_33bnw_16bnw_125hz.data', cov=True)
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140429-1429/stim300_filter_33bnw_16bnw_125hz.data', cov=True)

# Time stamp tests
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140502_vicon_timestamp/20140502-1958/stim300_filter_33bnw_16bnw_125hz.data', cov=True)
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140502_vicon_timestamp/20140502-2009/stim300_filter_33bnw_16bnw_125hz.data', cov=True)
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140502_vicon_timestamp/20140502-2012/stim300_filter_33bnw_16bnw_125hz.data', cov=True)
#imuOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140502_vicon_timestamp/20140502-2015/stim300_filter_33bnw_16bnw_125hz.data', cov=True)
imuOrient.readData('/home/jhidalgocarrio/exoter/experiments/20140600_pink_odometry_test/20140605-1731/data/stim300_attitude.data', cov=True)

imuOrient.eigenValues()

# Read the vicon orientation information
viconOrient = data.QuaternionData()
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140422-1505/vicon_processing_100hz.data', cov=False)
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140422-1539/vicon_processing_100hz.data', cov=False)
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140423-1923/vicon_processing_100hz.data', cov=False)
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140423-1942/vicon_processing_100hz.data', cov=False)
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140425-1923/vicon_processing_100hz.data', cov=False)
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140422_stim300_vs_vicon/20140429-1429/vicon_processing_100hz.data', cov=False)

# Time stamp tests
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140502_vicon_timestamp/20140502-1958/vicon_processing_100hz_with_timestamp.data', cov=False)
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140502_vicon_timestamp/20140502-2009/vicon_processing_100hz_without_timestamp.data', cov=False)
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140502_vicon_timestamp/20140502-2012/vicon_processing_100hz_without_timestamp_wifi.data', cov=False)
#viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140502_vicon_timestamp/20140502-2015/vicon_processing_100hz_with_timestamp_wifi.data', cov=False)
#viconOrient.readData('/home/jhidalgocarrio/exoter/experiments/20140600_pink_odometry_test/20140605-1731/data/vicon_attitude.data', cov=False)
viconOrient.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140715-1618_vicon_uncertainty/pose_ref_orientation.0.data', cov=True)
viconOrient.eigenValues()

#Plotting Orientation values
plt.figure(1)
time = imuOrient.t
euler = []
euler.append(imuOrient.getEuler(2))# Roll
euler.append(imuOrient.getEuler(1))# Pitch
euler.append(imuOrient.getEuler(0))# Yaw

euler[2] = euler[2] - euler[2][0] # set yaw staring at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

plt.plot(time, euler[0], marker='.', label="IMU Roll", color=[1.0,0,0], alpha=0.5, lw=2)
#plt.plot(time, euler[1], marker='.', label="IMU Pitch", color=[0,1.0,0], alpha=0.5, lw=2)
#plt.plot(time, euler[2], marker='.', label="IMU Yaw", color=[0,0,1.0], alpha=0.5, lw=2)


# Orientation Std
stdeulerpos=[];
for i in range(0,len(time)):
    stdeulerpos.append(euler[0][i] + imuOrient.getStd(axis=0, levelconf=3)[i])

stdeulerneg=[];
for i in range(0,len(time)):
    stdeulerneg.append(euler[0][i] - imuOrient.getStd(axis=0, levelconf=3)[i])

plt.plot(time, stdeulerpos, marker='.', color="grey", alpha=1.0, lw=1)
plt.plot(time, stdeulerneg, marker='.', color="grey", alpha=1.0, lw=1)


time = viconOrient.t
euler = []
euler.append(viconOrient.getEuler(2))# Roll
euler.append(viconOrient.getEuler(1))# Pitch
euler.append(viconOrient.getEuler(0))# Yaw

euler[2] = euler[2] - euler[2][0] #set yaw starting at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

plt.plot(time, euler[0], marker='.', label="Vicon Roll", color=[0.7,0,0], alpha=0.5, lw=2)
#plt.plot(time, euler[1], marker='.', label="Vicon Pitch", color=[0,0.5,0], alpha=0.5, lw=2)
#plt.plot(time, euler[2], marker='.', label="Vicon Yaw", color=[0,0.4,0.5], alpha=0.5, lw=2)

# Orientation Std
stdeulerpos=[];
for i in range(0,len(time)):
    stdeulerpos.append(euler[0][i] +  np.nan_to_num(viconOrient.getStd(axis=0, levelconf=3)[i]))

stdeulerneg=[];
for i in range(0,len(time)):
    stdeulerneg.append(euler[0][i] - np.nan_to_num(viconOrient.getStd(axis=0, levelconf=3)[i]))


plt.plot(time, stdeulerpos, marker='.', label="Std (+)", color="grey", alpha=1.0, lw=1)
plt.plot(time, stdeulerneg, marker='.', label="Std (-)", color="grey", alpha=1.0, lw=1)


plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Angle [${}^\circ$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

#Plotting Orientation values with variable line width
from matplotlib.collections import LineCollection
fig = plt.figure(2)
ax = fig.add_subplot(111)

time = imuOrient.t
euler = []
euler.append(imuOrient.getEuler(2))# Roll
euler.append(imuOrient.getEuler(1))# Pitch
euler.append(imuOrient.getEuler(0))# Yaw

euler[2] = euler[2] - euler[2][0] # set yaw staring at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

ax.plot(time, euler[0], marker='.', label="IMU Roll", color=[1.0,0,0], alpha=0.5, lw=2)

time = viconOrient.t
euler = []
euler.append(viconOrient.getEuler(2))# Roll
euler.append(viconOrient.getEuler(1))# Pitch
euler.append(viconOrient.getEuler(0))# Yaw

euler[2] = euler[2] - euler[2][0] #set yaw starting at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees


#Line width
uncer = np.nan_to_num(np.array(viconOrient.getStd(axis=0, levelconf=3))).real
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
for i in range(0,len(imuOrient.t)-1):
    #print time[i]
    timu = float(imuOrient.t[i+1]) - float(imuOrient.t[i])
    deltaimu.append(timu)

deltaimu_t = mean(deltaimu)


deltavicon = []
for i in range(0,len(viconOrient.t)-1):
    #print time[i]
    tvicon = float(viconOrient.t[i+1]) - float(viconOrient.t[i])
    deltavicon.append(tvicon)

deltavicon_t = mean(deltavicon)




from matplotlib.collections import LineCollection
x=linspace(0,4*pi,373)
y=cos(x)
y=euler[0][0:373]
temp = np.nan_to_num(np.array(viconOrient.getStd(axis=0, levelconf=3)))[:-1].real[0:373]
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
