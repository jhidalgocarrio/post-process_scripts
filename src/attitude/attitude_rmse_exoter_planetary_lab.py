#!/usr/bin/env python

path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141025-0005/'

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

reference_euler = []
reference_euler.append(reference_orient.getEuler(2))# Roll
reference_euler.append(reference_orient.getEuler(1))# Pitch
reference_euler.append(reference_orient.getEuler(0))# Yaw
reference_euler = np.asarray(reference_euler)


temindex = np.where(np.fabs(reference_euler[0]) > (30.0 * math.pi / 180.00))
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

#############################
# REFERENCE IN EULER ANGLES #
#############################
reference_euler = []
reference_euler.append(reference_orient.getEuler(2))# Roll
reference_euler.append(reference_orient.getEuler(1))# Pitch
reference_euler.append(reference_orient.getEuler(0))# Yaw
reference_euler = np.asarray(reference_euler)

#Alignment the ground truth
alignment_diff = []
alignment_diff.append(odometry_orient.getEuler(2)[0::20][118] - reference_orient.getEuler(2)[0::20][118]) # Roll
alignment_diff.append(odometry_orient.getEuler(1)[0::20][118] - reference_orient.getEuler(1)[0::20][118]) # Pitch
alignment_diff.append(odometry_orient.getEuler(0)[0::20][118] - reference_orient.getEuler(0)[0::20][118]) # Yaw
alignment_diff = np.asarray(alignment_diff)

reference_euler[0] = reference_euler[0] + alignment_diff[0]
reference_euler[1] = reference_euler[1] + alignment_diff[1]
reference_euler[2] = reference_euler[2] + alignment_diff[2]

# REFERENCE TIME
reference_time = reference_orient.atime

############################
# RMSE ALLAN VAR FILTER VERSUS GROUND TRUTH
#############################
time = imu_orient.atime
orient_euler = []
orient_euler.append(imu_orient.getEuler(2))# Roll
orient_euler.append(imu_orient.getEuler(1))# Pitch
orient_euler.append(imu_orient.getEuler(0))# Yaw
orient_euler = np.asarray(orient_euler)

# Compute the RMSE
imu_orient_rmse = []
imu_orient_rmse.append(sqrt(((orient_euler[0][1300:len(orient_euler[0])] - reference_euler[0][1300:len(orient_euler[0])]) ** 2).mean()))
imu_orient_rmse.append(sqrt(((orient_euler[1][1300:len(orient_euler[0])] - reference_euler[1][1300:len(orient_euler[0])]) ** 2).mean()))
imu_orient_rmse.append(sqrt(((orient_euler[2][1300:len(orient_euler[0])] - reference_euler[2][1300:len(orient_euler[0])]) ** 2).mean()))
imu_orient_rmse = np.asarray(imu_orient_rmse)

# Convert to degrees
imu_orient_rmse = imu_orient_rmse * 180.00/math.pi# Convert to degrees

