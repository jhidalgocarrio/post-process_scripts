#!/usr/bin/env python

#path = '/home/javi/exoter/development/data/20141023_pink_test/20141023-2011/'
path = '/home/javi/exoter/development/data/20140000_gaussian_processes/merged/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

pose_ref_position_file =  path + 'delta_pose_ref_position.0.data'

pose_odo_position_file =  path + 'delta_pose_odo_position.0.data'

pose_ref_velocity_file =  path + 'delta_pose_ref_velocity.0.data'

pose_odo_velocity_file =  path + 'delta_pose_odo_velocity.0.data'

pose_imu_orientation_file =  path + 'pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data'
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

from sklearn.gaussian_process import GaussianProcess
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel

# Reference Robot Velocity
reference_position = data.ThreeData()
reference_position.readData(pose_ref_position_file, cov=True)

# Odometry Robot Velocity
odometry_position = data.ThreeData()
odometry_position.readData(pose_odo_position_file, cov=True)

# Reference Robot Velocity
reference_velocity = data.ThreeData()
reference_velocity.readData(pose_ref_velocity_file, cov=True)

# Odometry Robot Velocity
odometry_velocity = data.ThreeData()
odometry_velocity.readData(pose_odo_velocity_file, cov=True)

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
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference_velocity.data[:,0]))
temindex = np.asarray(temindex)

reference_position.delete(temindex)
odometry_position.delete(temindex)
reference_velocity.delete(temindex)
odometry_velocity.delete(temindex)
imu_orient.delete(temindex)
imu_gyro.delete(temindex)
imu_acc.delete(temindex)
robot_joints.delete(temindex)


#########################
## LOAD INPUT TEST    ##
#########################

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

###########################
# Create Reference Output #

#reference = np.column_stack((reference_position.getAxis(0), reference_position.getAxis(1), reference_position.getAxis(2)))
reference = np.column_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2)))

#odometry = np.column_stack((odometry_position.getAxis(0), odometry_position.getAxis(1), odometry_position.getAxis(2)))
odometry = np.column_stack((odometry_velocity.getAxis(0), odometry_velocity.getAxis(1), odometry_velocity.getAxis(2)))


#########################
## SPLIT INPUT TEST    ##
#########################
sampling_frequency = 1.0/mean(reference_velocity.delta[0:100])
size_block = 5 * sampling_frequency
number_blocks = int(len(reference_velocity.delta)/size_block)

# Split joints (one joint info per column)
joints, jointstd = data.input_reduction(joints, number_blocks)

# Split inertia (one axis info per column)
inertia, inertiastd = data.input_reduction(inertia, number_blocks)

# Split orientation (one axis info per column)
orient, orientstd = data.input_reduction(orient, number_blocks)

# Split reference (one axis info per column)
reference, referencestd = data.input_reduction(reference, number_blocks)

# Split odometry (one axis info per column)
odometry, odometrystd = data.input_reduction(odometry, number_blocks)

error = np.absolute(reference - odometry)

###################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(2)
ax = fig.add_subplot(111)
t = np.linspace(0, reference.shape[0], reference.shape[0])
plt.rc('text', usetex=False)# activate latex text rendering

ax.plot(t, error[:,0], 'r:', label=u'Error')
ax.plot(t, odometry[:,0], 'b-', label=u'Odometry')
ax.plot(t, reference[:,0], 'g-', label=u'Reference')

plt.xlabel(r'Samples', fontsize=35, fontweight='bold')
plt.ylabel(r'f(x) [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

#########################
## GAUSSIAN PROCESS    ##
#########################

# GP Multidimensional Input
X = np.column_stack((joints, inertia[:,3:6], orient))

# GP onedimensional Output
Y =  error[:,0]
dY = np.absolute(referencestd[:,0] + odometrystd[:,0]) * 0.001

#kernel = 0.01 * RBF(l=0.5)
#gp = GaussianProcessRegressor(kernel=kernel, sigma_squared_n=0.0)

gp = GaussianProcess(corr='squared_exponential', theta0=1e-1,
                     thetaL=1e-3, thetaU=1,
                     nugget=(dY / Y) ** 2,
                     random_start=100)
gp.fit(X, Y)

###################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(3)
ax = fig.add_subplot(111)
t = np.linspace(0, Y.shape[0], Y.shape[0])
plt.rc('text', usetex=False)# activate latex text rendering
ax.plot(t, error[:,0], 'r:', label=u'Error')
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



###################
## PREDICTION    ##
###################
path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

pose_ref_position_file =  path + 'delta_pose_ref_position.0.data'

pose_odo_position_file =  path + 'delta_pose_odo_position.0.data'

pose_ref_velocity_file =  path + 'delta_pose_ref_velocity.0.data'

pose_odo_velocity_file =  path + 'delta_pose_odo_velocity.0.data'

pose_imu_orientation_file =  path + 'pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data'
#######################################

#############################
## LOAD EVALUATION TEST    ##
#############################

# Reference Robot Position
reference_position = data.ThreeData()
reference_position.readData(pose_ref_position_file, cov=True)

# Odometry Robot Position
odometry_position = data.ThreeData()
odometry_position.readData(pose_odo_position_file, cov=True)

# Reference Robot Velocity
reference_velocity = data.ThreeData()
reference_velocity.readData(pose_ref_velocity_file, cov=True)

# Odometry Robot Velocity
odometry_velocity = data.ThreeData()
odometry_velocity.readData(pose_odo_velocity_file, cov=True)

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
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference_velocity.data[:,0]))
temindex = np.asarray(temindex)

reference_position.delete(temindex)
odometry_position.delete(temindex)
reference_velocity.delete(temindex)
odometry_velocity.delete(temindex)
imu_orient.delete(temindex)
imu_gyro.delete(temindex)
imu_acc.delete(temindex)
robot_joints.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
reference_velocity.eigenValues()
odometry_velocity.eigenValues()
imu_orient.eigenValues()
imu_acc.eigenValues()
imu_gyro.eigenValues()

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

#reference = np.column_stack((reference_position.getAxis(0), reference_position.getAxis(1), reference_position.getAxis(2)))
reference = np.column_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2)))

#odometry = np.column_stack((odometry_position.getAxis(0), odometry_position.getAxis(1), odometry_position.getAxis(2)))
odometry = np.column_stack((odometry_velocity.getAxis(0), odometry_velocity.getAxis(1), odometry_velocity.getAxis(2)))

######################
# Split joints (one joint info per column)
#
sampling_frequency = 1.0/mean(reference_velocity.delta[0:100])
size_block = 5 * sampling_frequency
number_blocks = int(len(reference_velocity.delta)/size_block)
#number_blocks = len(robot_joints.time)


joints, jointstd = data.input_reduction(joints, number_blocks)

# Split inertia (one axis info per column)
inertia, inertiastd = data.input_reduction(inertia, number_blocks)

# Split orientation (one axis info per column)
orient, orientstd = data.input_reduction(orient, number_blocks)

# Split reference (one axis info per column)
reference, referencestd = data.input_reduction(reference, number_blocks)

# Split odometry (one axis info per column)
odometry, odometrystd = data.input_reduction(odometry, number_blocks)


###########################
# GP Multidimensional vector
Xp = np.column_stack((joints, inertia[:,3:6], orient))

###################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = odometry_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
xvelocity = odometry[:,0]
ax.plot(time, xvelocity, marker='o', linestyle='-.', label="Odometry Velocity", color=[0.0,0.0,1.0], lw=2)

time = reference_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
xvelocity = reference[:,0]
ax.scatter(time, xvelocity, marker='D', label="Reduced Reference", color=[1.0,0.0,0.0], s=80)
ax.plot(time, xvelocity, marker='D', linestyle='--', label="Reduced Reference", color=[1.0,0.0,0.0], lw=2)

#meanxp, covxp = gp.predict(Xp, return_cov=True)
meanxp, covxp = gp.predict(Xp, eval_MSE=True)
time = odometry_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
xvelocity = meanxp #odometry[:,0]
sigma = np.sqrt(covxp)
sigma = 1.0 * sigma
ax.plot(time, xvelocity, marker='None', linestyle='-', label="GP Velocity", color=[0.0,1.0,0.0], lw=4)
ax.fill_between(time, xvelocity - sigma, xvelocity + sigma, alpha=0.5, color='k')

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)



