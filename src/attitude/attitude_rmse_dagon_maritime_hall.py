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

########################
### REMOVE OUTLIERS  ###
########################
reference_euler = []
reference_euler.append(reference_orient.getEuler(2))# Roll
reference_euler.append(reference_orient.getEuler(1))# Pitch
reference_euler.append(reference_orient.getEuler(0))# Yaw
reference_euler = np.asarray(reference_euler)

temindex = np.where(np.fabs(reference_euler[2]) > (5.0 * math.pi / 180.00))
temindex = np.asarray(temindex)

reference_orient.delete(temindex)
ikf_orient.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
ikf_orient.covSymmetry()
ikf_orient.eigenValues()

reference_orient.covSymmetry()
reference_orient.eigenValues()

#############################
# REFERENCE IN EULER ANGLES #
#############################
reference_euler = []
reference_euler.append(reference_orient.getEuler(2))# Roll
reference_euler.append(reference_orient.getEuler(1))# Pitch
reference_euler.append(reference_orient.getEuler(0))# Yaw
reference_euler = np.asarray(reference_euler)

# REFERENCE TIME
reference_time = reference_orient.atime

#############################
# RMSE ALLAN VAR FILTER VERSUS GROUND TRUTH
#############################
time = ikf_orient.atime
orient_euler = []
orient_euler.append(ikf_orient.getEuler(2))# Roll
orient_euler.append(ikf_orient.getEuler(1))# Pitch
orient_euler.append(ikf_orient.getEuler(0))# Yaw
orient_euler = np.asarray(orient_euler)

# Compute the RMSE
ikf_orient_rmse = []
ikf_orient_rmse.append(sqrt(((orient_euler[0] - reference_euler[0]) ** 2).mean()))
ikf_orient_rmse.append(sqrt(((orient_euler[1] - reference_euler[1]) ** 2).mean()))
ikf_orient_rmse.append(sqrt(((orient_euler[2] - reference_euler[2]) ** 2).mean()))
ikf_orient_rmse = np.asarray(ikf_orient_rmse)

# Convert to degrees
ikf_orient_rmse = ikf_orient_rmse * 180.00/math.pi# Convert to degrees


