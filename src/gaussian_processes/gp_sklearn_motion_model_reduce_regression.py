#!/usr/bin/env python

#path = '/home/javi/exoter/development/data/20141023_pink_test/20141023-2011/'
path = '/home/javi/exoter/development/data/20140000_gaussian_processes/merged/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

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

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel

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
reference = np.column_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2)))
odometry = np.column_stack((odometry_velocity.getAxis(0), odometry_velocity.getAxis(1), odometry_velocity.getAxis(2)))

#odometry_mod = odometry + (mean(imu_acc.delta[0:100])*inertia[0:odometry.shape[0],0:3])
#odometry_int = np.cumsum((mean(imu_acc.delta[0:100])*inertia[0:odometry.shape[0],0:1]))
#########################
## SPLIT INPUT TEST    ##
#########################
def input_reduction (input_array = None, number_blocks = 0.0):
    if input_array is None:
        raise ValueError("Input cannot be None")
    if number_blocks is 0.0:
        raise ValueError("Cannot split array with zero number of blocks")

    input_array_shape = 0
    if len(input_array.shape) == 2:
        input_array_shape = input_array.shape[1]
    else:
        input_array_shape = 1

    new_input = np.ndarray(shape = (number_blocks, input_array_shape))
    std_input = np.ndarray(shape = (number_blocks, input_array_shape))

    for i in range(input_array_shape):
        if input_array_shape > 1:
            split_array = np.array_split(input_array[:,i], number_blocks)
        else:
            split_array = np.array_split(input_array, number_blocks)

        mean_array = np.ndarray(len(split_array), dtype=double)
        std_array = np.ndarray(len(split_array), dtype=double)

        for j in range(len(split_array)):
            mean_array[j] = mean(split_array[j])
            std_array[j] = std(split_array[j])

        if input_array_shape > 1:
            new_input[:,i] = mean_array
            std_input[:,i] = std_array
        else:
            new_input = mean_array
            std_input = std_array

    return new_input, std_input
#########################

sampling_frequency = 1.0/mean(reference_velocity.delta[0:100])
size_block = 5 * sampling_frequency
number_blocks = int(len(reference_velocity.delta)/size_block)

# Split joints (one joint info per column)
joints, jointstd = input_reduction(joints, number_blocks)

# Split inertia (one axis info per column)
inertia, inertiastd = input_reduction(inertia, number_blocks)

# Split orientation (one axis info per column)
orient, orientstd = input_reduction(orient, number_blocks)

# Split reference (one axis info per column)
reference, referencestd = input_reduction(reference, number_blocks)

# Split odometry (one axis info per column)
odometry, odometrystd = input_reduction(odometry, number_blocks)

#########################
## GAUSSIAN PROCESS    ##
#########################

# GP Multidimensional Input
X = np.column_stack((joints, inertia[:,3:6], orient))

# GP onedimensional Output
Y =  reference[:,0]

kernel = 1.0 * RBF(l=1.0) + WhiteKernel(c=1e-6, c_bounds=(1e-10, 1e-1))
gp = GaussianProcessRegressor(kernel=kernel, sigma_squared_n=0.0).fit(X, Y)

# Plot LML landscape
#plt.figure(2)
#theta0 = np.logspace(-2, 0, 49)
#theta1 = np.logspace(-2, 0, 50)
#Theta0, Theta1 = np.meshgrid(theta0, theta1)
#LML = [[gp.log_marginal_likelihood([0.36, Theta0[i, j], Theta1[i, j]])
#        for i in range(Theta0.shape[0])] for j in range(Theta0.shape[1])]
#LML = np.array(LML).T
#
#vmin, vmax = (-LML).min(), (-LML).max()
#vmax = 50
#plt.contour(Theta0, Theta1, -LML,
#            levels=np.logspace(np.log10(vmin), np.log10(vmax), 50),
#            norm=LogNorm(vmin=vmin, vmax=vmax))
#plt.colorbar()
#plt.xscale("log")
#plt.yscale("log")
#plt.xlabel("Length-scale")
#plt.ylabel("Noise-level")
#plt.title("Log-marginal-likelihood")
#plt.tight_layout()
#plt.show(block=False)

###################
## PREDICTION    ##
###################
path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

pose_ref_velocity_file =  path + 'delta_pose_ref_velocity.0.data'

pose_odo_velocity_file =  path + 'delta_pose_odo_velocity.0.data'

pose_imu_orientation_file =  path + 'pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data'
#######################################

#############################
## LOAD EVALUATION TEST    ##
#############################

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

reference = np.column_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2)))

odometry = np.column_stack((odometry_velocity.getAxis(0), odometry_velocity.getAxis(1), odometry_velocity.getAxis(2)))

######################
# Split joints (one joint info per column)
#
sampling_frequency = 1.0/mean(reference_velocity.delta[0:100])
size_block = 5 * sampling_frequency
number_blocks = int(len(reference_velocity.delta)/size_block)
#number_blocks = len(robot_joints.time)


joints, jointstd = input_reduction(joints, number_blocks)

# Split inertia (one axis info per column)
inertia, inertiastd = input_reduction(inertia, number_blocks)

# Split orientation (one axis info per column)
orient, orientstd = input_reduction(orient, number_blocks)

# Split reference (one axis info per column)
reference, referencestd = input_reduction(reference, number_blocks)

# Split odometry (one axis info per column)
odometry, odometrystd = input_reduction(odometry, number_blocks)


###########################
# GP Multidimensional vector
Xp = np.column_stack((joints, inertia[:,3:6], orient))

###################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = odometry_velocity.time
time, timestd = input_reduction(time, number_blocks)
xvelocity = odometry[:,0]
ax.plot(time, xvelocity, marker='o', linestyle='-.', label="Odometry Velocity", color=[0.0,0.0,1.0], lw=2)

time = reference_velocity.time
time, timestd = input_reduction(time, number_blocks)
xvelocity = reference[:,0]
ax.scatter(time, xvelocity, marker='D', label="Reduced Reference", color=[1.0,0.0,0.0], s=80)

meanxp, covxp = gp.predict(Xp, return_cov=True)
time = odometry_velocity.time
time, timestd = input_reduction(time, number_blocks)
xvelocity = meanxp
sigma = np.sqrt(np.diag(covxp))
#sigma = 2.0 * sigma
ax.plot(time, xvelocity, marker='None', linestyle='-', label="GP Velocity", color=[0.0,1.0,0.0], lw=4)
plt.fill_between(time, xvelocity - sigma, xvelocity + sigma, alpha=0.5, color='k')

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


###################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(2)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = odometry_velocity.time
time, timestd = input_reduction(time, number_blocks)
xvelocity = odometry[:,0]
ax.plot(time, xvelocity, marker='o', linestyle='-.', label="Odometry Velocity", color=[0.0,0.0,1.0], lw=2)

time = reference_velocity.time
time, timestd = input_reduction(time, number_blocks)
xvelocity = reference[:,0]
ax.scatter(time, xvelocity, marker='D', label="Reduced Reference", color=[1.0,0.0,0.0], s=80)

meanxp, covxp = gp.predict(Xp, return_cov=True)
time = odometry_velocity.time
time, timestd = input_reduction(time, number_blocks)
xvelocity = odometry[:,0]
sigma =  odometry[:,0] - meanxp
sigma = 2.0 * sigma
#ax.plot(time, xvelocity, marker='None', linestyle='-', label="GP Velocity", color=[0.0,1.0,0.0], lw=4)
ax.fill_between(time, xvelocity - sigma, xvelocity + sigma, alpha=0.4, where=sigma <= 0.02, color='k', label='95% confidence interval')
ax.fill_between(time, xvelocity - sigma, xvelocity + sigma, alpha=0.4, where=sigma > 0.02, color='r', label='95% confidence interval')

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)



