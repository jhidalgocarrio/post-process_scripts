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
viconOrient.readData('/home/jhidalgocarrio/exoter/experiments/20140600_pink_odometry_test/20140605-1731/data/vicon_attitude.data', cov=False)
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

time = viconOrient.t
euler = []
euler.append(viconOrient.getEuler(2))# Roll
euler.append(viconOrient.getEuler(1))# Pitch
euler.append(viconOrient.getEuler(0))# Yaw

euler[2] = euler[2] - euler[2][0] #set yaw staring at zero

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

plt.plot(time, euler[0], marker='.', label="Vicon Roll", color=[0.7,0,0], alpha=0.5, lw=2)
#plt.plot(time, euler[1], marker='.', label="Vicon Pitch", color=[0,0.5,0], alpha=0.5, lw=2)
#plt.plot(time, euler[2], marker='.', label="Vicon Yaw", color=[0,0.4,0.5], alpha=0.5, lw=2)

plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Angle [${}^\circ$]')
plt.grid(True)
plt.legend(prop={'size':25})
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

