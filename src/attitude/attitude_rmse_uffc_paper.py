#!/usr/bin/env python

path = '/home/javi/exoter/development/data/20150323_ww_dlr/imu_stim300_attitude_test_20150325-1716/'

##################################
pose_ikf_orient_file = path + 'pose_ikf_orientation.3.data'

pose_ikf_inflated_coef_orient_file = path + 'pose_ikf_inflated_coef_orientation.4.data'

#pose_ikf_data_sheet_coef_orient_file = path + 'pose_ikf_data_sheet_coef_orientation.0.data'
pose_ikf_data_sheet_coef_orient_file = path + 'pose_ikf_incomplete_model.1.data'

pose_imu_orient_file = path + 'pose_imu_orientation.0.data'

pose_ref_orient_file = path + 'pose_ref_orientation.0.data'
##################################

import sys
sys.path.insert(0, './src/core')
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
import datadisplay as data

# Read the ikf filter orientation information
ikf_orient = data.QuaternionData()
ikf_orient.readData(pose_ikf_orient_file, cov=True)

# Read the inflated ikf filter orientation information
ikf_inflated_coef_orient = data.QuaternionData()
ikf_inflated_coef_orient.readData(pose_ikf_inflated_coef_orient_file, cov=True)

# Read the data sheet values for ikf filter orientation information
ikf_data_sheet_coef_orient = data.QuaternionData()
ikf_data_sheet_coef_orient.readData(pose_ikf_data_sheet_coef_orient_file, cov=True)

# Read the imu orientation information
imu_orient = data.QuaternionData()
imu_orient.readData(pose_imu_orient_file, cov=True)

# Read the reference orientation information
reference_orient = data.QuaternionData()
reference_orient.readData(pose_ref_orient_file, cov=False)

########################
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference_orient.data))
temindex = np.asarray(temindex)

ikf_orient.delete(temindex)
ikf_inflated_coef_orient.delete(temindex)
ikf_data_sheet_coef_orient.delete(temindex)
imu_orient.delete(temindex)
reference_orient.delete(temindex)

#############################
# REFERENCE IN EULER ANGLES #
#############################
reference_euler = []
reference_euler.append(reference_orient.getEuler(0))# Yaw
reference_euler.append(reference_orient.getEuler(1))# Pitch
reference_euler.append(reference_orient.getEuler(2))# Roll
reference_euler = np.asarray(reference_euler)

# REFERENCE TIME
reference_time = reference_orient.atime

#############################
# RMSE ALLAN VAR FILTER VERSUS GROUND TRUTH
#############################
time = ikf_orient.atime
orient_euler = []
orient_euler.append(ikf_orient.getEuler(0))# Yaw
orient_euler.append(ikf_orient.getEuler(1))# Pitch
orient_euler.append(ikf_orient.getEuler(2))# Roll
orient_euler = np.asarray(orient_euler)

# INTERPOLATION
euler=[]
euler.append(np.interp(reference_time, time, orient_euler[0]))
euler.append(np.interp(reference_time, time, orient_euler[1]))
euler.append(np.interp(reference_time, time, orient_euler[2]))
euler = np.asarray(euler)


# Alignment with ground truth
alignment_diff = []
alignment_diff.append(ikf_orient.getEuler(0)[0] - reference_orient.getEuler(0)[0]) # Yaw
alignment_diff.append(-ikf_orient.getEuler(1)[0] - reference_orient.getEuler(1)[0]) # Pitch
alignment_diff.append(-ikf_orient.getEuler(2)[0] - reference_orient.getEuler(2)[0]) # Roll
alignment_diff = np.asarray(alignment_diff)

# Align the starting point
euler[0] = euler[0] - alignment_diff[0]
euler[1] = euler[1] + alignment_diff[1]
euler[2] = euler[2] - alignment_diff[2]

# IMU frame is 180 rotates wrt body
euler[1] = -euler[1]
euler[2] = -euler[2]

# Check the heading to be -180, 180
for i in range(0, len(euler[0])-1):
    if euler[0][i] > (2.0*math.pi):
        euler[0][i] = euler[0][i] - (2*math.pi)

    if euler[0][i] > math.pi:
        euler[0][i] = -math.pi + (euler[0][i] - math.pi)

    if euler[0][i] < -(2.0 * math.pi):
        euler[0][i] = euler[0][i] + (2.0*math.pi)

    if euler[0][i] < -math.pi:
        euler[0][i] = math.pi + (math.pi + euler[0][i])


# Compute the RMSE
ikf_orient_rmse = []
ikf_orient_rmse.append(sqrt(((euler[0] - reference_euler[0]) ** 2).mean()))
ikf_orient_rmse.append(sqrt(((euler[1] - reference_euler[1]) ** 2).mean()))
ikf_orient_rmse.append(sqrt(((euler[2] - reference_euler[2]) ** 2).mean()))
ikf_orient_rmse = np.asarray(ikf_orient_rmse)

# Convert to degrees
ikf_orient_rmse = ikf_orient_rmse * 180.00/math.pi# Convert to degrees

#############################
# RMSE DATA SHEET FILTER VERSUS GROUND TRUTH
#############################
time = ikf_data_sheet_coef_orient.atime
orient_euler = []
orient_euler.append(ikf_data_sheet_coef_orient.getEuler(0))# Yaw
orient_euler.append(ikf_data_sheet_coef_orient.getEuler(1))# Pitch
orient_euler.append(ikf_data_sheet_coef_orient.getEuler(2))# Roll
orient_euler = np.asarray(orient_euler)

# INTERPOLATION
euler=[]
euler.append(np.interp(reference_time, time, orient_euler[0]))
euler.append(np.interp(reference_time, time, orient_euler[1]))
euler.append(np.interp(reference_time, time, orient_euler[2]))
euler = np.asarray(euler)

# Alignment with ground truth
alignment_diff = []
alignment_diff.append(ikf_data_sheet_coef_orient.getEuler(0)[0] - reference_orient.getEuler(0)[0]) # Yaw
alignment_diff.append(-ikf_data_sheet_coef_orient.getEuler(1)[0] - reference_orient.getEuler(1)[0]) # Pitch
alignment_diff.append(-ikf_data_sheet_coef_orient.getEuler(2)[0] - reference_orient.getEuler(2)[0]) # Roll
alignment_diff = np.asarray(alignment_diff)

# Align the starting point
euler[0] = euler[0] - alignment_diff[0]
euler[1] = euler[1] + alignment_diff[1]
euler[2] = euler[2] - alignment_diff[2]

# IMU frame is 180 rotates wrt body
euler[1] = -euler[1]
euler[2] = -euler[2]

# Check the heading to be -180, 180
for i in range(0, len(euler[0])-1):
    if euler[0][i] > (2.0*math.pi):
        euler[0][i] = euler[0][i] - (2*math.pi)

    if euler[0][i] > math.pi:
        euler[0][i] = -math.pi + (euler[0][i] - math.pi)

    if euler[0][i] < -(2.0 * math.pi):
        euler[0][i] = euler[0][i] + (2.0*math.pi)

    if euler[0][i] < -math.pi:
        euler[0][i] = math.pi + (math.pi + euler[0][i])

# Compute the RMSE
ikf_data_sheet_coef_orient_rmse = []
ikf_data_sheet_coef_orient_rmse.append(sqrt(((euler[0] - reference_euler[0]) ** 2).mean()))
ikf_data_sheet_coef_orient_rmse.append(sqrt(((euler[1] - reference_euler[1]) ** 2).mean()))
ikf_data_sheet_coef_orient_rmse.append(sqrt(((euler[2] - reference_euler[2]) ** 2).mean()))
ikf_data_sheet_coef_orient_rmse = np.asarray(ikf_data_sheet_coef_orient_rmse)

# Convert to degrees
ikf_data_sheet_coef_orient_rmse = ikf_data_sheet_coef_orient_rmse * 180.00/math.pi# Convert to degrees

#############################
# RMSE INFLATED COVARIANCE FILTER VERSUS GROUND TRUTH
#############################
time = ikf_inflated_coef_orient.atime
orient_euler = []
orient_euler.append(ikf_inflated_coef_orient.getEuler(0))# Yaw
orient_euler.append(ikf_inflated_coef_orient.getEuler(1))# Pitch
orient_euler.append(ikf_inflated_coef_orient.getEuler(2))# Roll
orient_euler = np.asarray(orient_euler)

# INTERPOLATION
euler=[]
euler.append(np.interp(reference_time, time, orient_euler[0]))
euler.append(np.interp(reference_time, time, orient_euler[1]))
euler.append(np.interp(reference_time, time, orient_euler[2]))
euler = np.asarray(euler)

# Alignment with ground truth
alignment_diff = []
alignment_diff.append(ikf_inflated_coef_orient.getEuler(0)[0] - reference_orient.getEuler(0)[0]) # Yaw
alignment_diff.append(-ikf_inflated_coef_orient.getEuler(1)[0] - reference_orient.getEuler(1)[0]) # Pitch
alignment_diff.append(-ikf_inflated_coef_orient.getEuler(2)[0] - reference_orient.getEuler(2)[0]) # Roll
alignment_diff = np.asarray(alignment_diff)

# Align the starting point
euler[0] = euler[0] - alignment_diff[0]
euler[1] = euler[1] + alignment_diff[1]
euler[2] = euler[2] - alignment_diff[2]

# IMU frame is 180 rotates wrt body
euler[1] = -euler[1]
euler[2] = -euler[2]

# Check the heading to be -180, 180
for i in range(0, len(euler[0])-1):
    if euler[0][i] > (2.0*math.pi):
        euler[0][i] = euler[0][i] - (2*math.pi)

    if euler[0][i] > math.pi:
        euler[0][i] = -math.pi + (euler[0][i] - math.pi)

    if euler[0][i] < -(2.0 * math.pi):
        euler[0][i] = euler[0][i] + (2.0*math.pi)

    if euler[0][i] < -math.pi:
        euler[0][i] = math.pi + (math.pi + euler[0][i])

# Compute the RMSE
ikf_inflated_coef_orient_rmse = []
ikf_inflated_coef_orient_rmse.append(sqrt(((euler[0] - reference_euler[0]) ** 2).mean()))
ikf_inflated_coef_orient_rmse.append(sqrt(((euler[1] - reference_euler[1]) ** 2).mean()))
ikf_inflated_coef_orient_rmse.append(sqrt(((euler[2] - reference_euler[2]) ** 2).mean()))
ikf_inflated_coef_orient_rmse = np.asarray(ikf_inflated_coef_orient_rmse)

# Convert to degrees
ikf_inflated_coef_orient_rmse = ikf_inflated_coef_orient_rmse * 180.00/math.pi# Convert to degrees



