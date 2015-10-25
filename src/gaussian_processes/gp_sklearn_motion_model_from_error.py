#!/usr/bin/env python

path ='/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034_threed_odometry_with_gp/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

pose_imu_orientation_file =  path + 'pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data'

gp_odo_velocity_file = path + 'delta_pose_gp_odo_velocity.0.data'
#######################################

import sys
sys.path.insert(0, './src/core')
import numpy as np
from pylab import *
from matplotlib import pyplot as plt
from matplotlib.colors import LogNorm
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov
import joints as js
from numpy import linalg as la

from sklearn.gaussian_process import GaussianProcess
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel

# IMU orientation
imu_orient = data.QuaternionData()
imu_orient.readData(pose_imu_orientation_file, cov=True)

# IMU acceleration
imu_acc = data.ThreeData()
imu_acc.readData(pose_imu_acceleration_file, cov=False)

# IMU Angular Velocity
imu_gyro = data.ThreeData()
imu_gyro.readData(pose_imu_angular_velocity_file, cov=False)

# Robot Joints Position and Speed
names = "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"

robot_joints = js.Joints(names)
robot_joints.readData(joints_position_file, joints_speed_file)

########################
# Odometry Robot Velocity
gp_odometry_velocity = data.ThreeData()
gp_odometry_velocity.readData(gp_odo_velocity_file, cov=True)

########################
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(gp_odometry_velocity.data[:,0]))
temindex = np.asarray(temindex)

imu_orient.delete(temindex)
imu_gyro.delete(temindex)
imu_acc.delete(temindex)
robot_joints.delete(temindex)
gp_odometry_velocity.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
imu_orient.eigenValues()
imu_acc.eigenValues()
imu_gyro.eigenValues()
gp_odometry_velocity.covSymmetry()
gp_odometry_velocity.eigenValues()


###################
## LOAD INPUT    ##
###################

np.random.seed(1)

#################
# Joints Inputs #
joints = np.column_stack((
                        robot_joints.getSpeed("fl_translation"),
                        robot_joints.getSpeed("fr_translation"),
                        robot_joints.getSpeed("ml_translation"),
                        robot_joints.getSpeed("mr_translation"),
                        robot_joints.getSpeed("rl_translation"),
                        robot_joints.getSpeed("rr_translation"),
                        robot_joints.getPosition("fl_steer"),
                        robot_joints.getPosition("fr_steer"),
                        robot_joints.getPosition("rl_steer"),
                        robot_joints.getPosition("rr_steer"),
                        robot_joints.getPosition("fl_walking"),
                        robot_joints.getPosition("fr_walking"),
                        robot_joints.getPosition("ml_walking"),
                        robot_joints.getPosition("mr_walking"),
                        robot_joints.getPosition("rl_walking"),
                        robot_joints.getPosition("rr_walking"),
                        robot_joints.getPosition("left_passive"),
                        robot_joints.getPosition("right_passive"),
                        robot_joints.getPosition("rear_passive")
                        ))
##################
# Inertia Inputs #
inertia = np.column_stack((imu_acc.getAxis(0), imu_acc.getAxis(1), imu_acc.getAxis(2),
                           imu_gyro.getAxis(0), imu_gyro.getAxis(1), imu_gyro.getAxis(2)))

######################
# Orientation Inputs #
# Roll and Pitch angles in that order #
orient = np.column_stack((imu_orient.getEuler(2), imu_orient.getEuler(1)))

###################
## LOAD OUTPUT   ##
###################
variance = np.column_stack((gp_odometry_velocity.var[:,0], gp_odometry_velocity.var[:,1], gp_odometry_velocity.var[:,2]))
std_deviation = np.column_stack((np.sqrt(variance[:,0]), np.sqrt(variance[:,1]),
        np.sqrt(variance[:,2])))

#########################
## SPLIT INPUT TEST    ##
#########################
sampling_frequency = 1.0/mean(gp_odometry_velocity.delta[0:100])
size_block = 4 * sampling_frequency
number_blocks = int(len(gp_odometry_velocity.delta)/size_block)

# Split joints (one joint info per column)
joints, jointstd = data.input_reduction(joints, number_blocks)

# Split inertia (one axis info per column)
inertia, inertiastd = data.input_reduction(inertia, number_blocks)

# Split orientation (one axis info per column)
orient, orientstd = data.input_reduction(orient, number_blocks)

# Split std deviation (one axis info per column)
deviation, deviationstd = data.input_reduction(std_deviation, number_blocks)

#########################
## GAUSSIAN PROCESS    ##
#########################

# GP Multidimensional Input
X = np.column_stack((joints, inertia[:,3:6], orient))

# GP onedimensional Output
Y =  deviation[:,0]
#Y = np.linspace(0, 0, num=deviation.shape[0])
dY =  np.absolute(deviationstd[:,0])

gp = GaussianProcess(corr='squared_exponential', theta0=1e-1,
                     thetaL=1e-3, thetaU=1,
                     nugget=(dY / Y) ** 2,
                     random_start=100)
gp.fit(X, Y)


matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(3)
ax = fig.add_subplot(111)
t = np.linspace(0, Y.shape[0], Y.shape[0])
plt.rc('text', usetex=False)# activate latex text rendering
ax.plot(t, deviation[:,0], 'r:', label=u'Odometry')
ax.errorbar(t, Y, dY, fmt='r.', markersize=10, label=u'Observations')
prediction, mse = gp.predict(X, eval_MSE=True)
sigma = np.sqrt(mse)
ax.plot(t, prediction, 'b-', label=u'Prediction')
ax.fill(np.concatenate([t, t[::-1]]),
        np.concatenate([prediction - 1.9600 * sigma,
                       (prediction + 1.9600 * sigma)[::-1]]),
        alpha=.5, fc='b', ec='None', label='95% confidence interval')

plt.xlabel(r'Samples', fontsize=35, fontweight='bold')
plt.ylabel(r'f(x) [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


