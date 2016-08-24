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
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov
import joints as js
import GPy

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
errorstd = np.absolute(referencestd + odometrystd)

#########################
## PLOT THE VALUES     ##
#########################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.rc('text', usetex=False)# activate latex text rendering

time = mean(reference_velocity.delta[0:100]) * r_[0:len(reference_velocity.time)]
time, timestd = data.input_reduction(time, number_blocks)
label_text = "Error Velocity"
color_value = [0.2,0.2,0.2]
ax.plot(time, error[:,0], marker='o', linestyle='-', label=label_text,
        color=color_value, lw=2)
ax.fill(np.concatenate([time, time[::-1]]),
        np.concatenate([error[:,0] - errorstd[:,0],
            (error[:,0] + errorstd[:,0])[::-1]]),
        alpha=.5, fc='0.50', ec='None', label='68% confidence interval')

ax.plot(time, odometry[:,0], 'b-', label=u'Odometry', lw=2)
ax.plot(time, reference[:,0], 'g-', label=u'Reference', lw=2)

plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Angle [${}^\circ$]')
plt.grid(True)
plt.legend(prop={'size':25}, loc=1)
plt.show(block=False)

###########################
# Plot the error as the std
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering

time = mean(reference_velocity.delta[0:100]) * r_[0:len(reference_velocity.time)]
time, timestd = data.input_reduction(time, number_blocks)
sigma = abs(error[:,0]) * np.ones(len(error[:,0]))
label_text = "Error Velocity"
color_value = [0.2,0.2,0.2]
#ax.plot(time, np.zeros(len(error[:,0])), marker='o', linestyle='-', label=label_text, color=color_value, lw=2)
ax.fill(np.concatenate([time, time[::-1]]),
        np.concatenate([np.zeros(len(error[:,0])) - sigma,
            (np.zeros(len(error[:,0])) + sigma)[::-1]]),
        alpha=.5, fc='0.50', ec='None', label='68% confidence interval')
color_value = [0.0,0.0,0.0]
ax.plot(time, -abs(error[:,0]), marker='o', linestyle='-', color=color_value, lw=1)
ax.plot(time, +abs(error[:,0]), marker='o', linestyle='-', color=color_value, lw=1)

plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Angle [${}^\circ$]')
plt.grid(True)
plt.legend(prop={'size':25}, loc=1)
plt.show(block=False)

###########################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering

time = mean(reference_velocity.delta[0:100]) * r_[0:len(reference_velocity.time)]
time, timestd = data.input_reduction(time, number_blocks)
label_text = "Reference Velocity"
color_value = [1.0,0,0]
ax.plot(time, reference[:,0], marker='o', linestyle='-', label=label_text, color=color_value, lw=6)

time = mean(odometry_velocity.delta[0:100]) * r_[0:len(odometry_velocity.time)]
time, timestd = data.input_reduction(time, number_blocks)
sigma = np.absolute(error) * np.ones(error.shape)
label_text = "Odometry Velocity"
color_value = [0.5,0.0,0]
ax.plot(time, odometry[:,0], marker='o', linestyle='-', label=label_text, color=color_value, lw=6)
ax.fill(np.concatenate([time, time[::-1]]),
        np.concatenate([odometry[:,0] - sigma[:,0],
            (odometry[:,0] + sigma[:,0])[::-1]]),
        alpha=.5, fc='0.50', ec='None', label='68% confidence interval')

plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Angle [${}^\circ$]')
plt.grid(True)
plt.legend(prop={'size':25}, loc=1)
plt.show(block=False)

#########################
## GAUSSIAN PROCESS    ##
#########################

# GP Multidimensional Input
X = np.column_stack((joints, inertia[:,0:2], inertia[:,3:6], orient))

# GP Multidimensional Output
sigma = np.absolute(error) * np.ones(error.shape)
Y =  np.column_stack((sigma))
Y = np.column_stack(Y)

# KERNEL
ker_rbf = GPy.kern.RBF(input_dim = X.shape[1], ARD=True) # with ARD one lengthscale parameter per dimension
ker =  ker_rbf

# GP MODEL
m = GPy.models.GPRegression(X, Y, kernel=ker)
print m
m.gradient # indication on understanding the (possibly hard) optimization process.

# OPTIMIZATION
m.optimize('bfgs', messages=True, max_f_eval=1000)
print m

###################
m.plot_f(ax=0) #Show the predictive values of the GP.
plt.errorbar(X[:,0],Y[:,0],yerr=errorstd[:,0],fmt=None,ecolor='r',zorder=1)
plt.grid()
plt.plot(X[:,0],Y[:,0],'kx',mew=1.5)
###################


###################
## PREDICTION    ##
###################
#path = '/home/javi/exoter/development/data/20141023_pink_test/20141023-2001/'
#path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141024-2317/'
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

###########################
# Create Reference Output #
reference = np.column_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2)))

odometry = np.column_stack((odometry_velocity.getAxis(0), odometry_velocity.getAxis(1), odometry_velocity.getAxis(2)))

######################
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

# Ground truth error
error = np.absolute(reference - odometry)

######################
# GP Multidimensional vector
Xp = np.column_stack((joints, inertia[:,0:2], inertia[:,3:6], orient))


###################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = odometry_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
xvelocity = odometry[:,0]
ax.plot(time, xvelocity, marker='o', linestyle='-.', label="Odometry velocity", color=[0.0,0.0,1.0], lw=2)

[meanxp, varxp] = m.predict(Xp)

sigma = 1.0 * abs(meanxp[0:len(xvelocity),:]+varxp[0:len(xvelocity),:])
ax.fill(np.concatenate([time, time[::-1]]),
        np.concatenate([xvelocity - sigma[:,0],
            (xvelocity + sigma[:,0])[::-1]]),
        alpha=.5, fc='0.50', ec='None', label='68% confidence interval')

ax.plot(time, meanxp[:,0], marker='<', linestyle='--', label="GP mean residual",
        color=[0.0,1.0,0.0], lw=2)
ax.plot(time, (meanxp[:,0]+2*varxp[:,0]).flatten(), linestyle='--',
        color=[0.0,1.0,0.0], lw=1.0)
ax.plot(time, (meanxp[:,0]-2*varxp[:,0]).flatten(), linestyle='--',
        color=[0.0,1.0,0.0], lw=1.0)

time = reference_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
xvelocity = reference[:,0]
ax.scatter(time, xvelocity, marker='D', label="Reference velocity", color=[1.0,0.0,0.0], s=80)
ax.plot(time, xvelocity, marker='D', linestyle='--', label="Reduced Reference", color=[1.0,0.0,0.0], lw=2)

ax.plot(time, error[:,0], marker='o', linestyle='-', label="Ground truth error",
        color=color_value, lw=2)

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


####### GP PREDICTION ####################
npts=8145
time = odometry_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
ti = np.linspace(min(time), max(time), npts)

# Interpolate GP residual 3-Dimensions
meanxp_inter = np.column_stack((
                    np.interp(ti, time, meanxp[:,0]),
                    np.interp(ti, time, meanxp[:,1]),
                    np.interp(ti, time, meanxp[:,2])
                    ))

# Interpolate GP uncertainty residual 3-Dimensions
varxp_inter = np.column_stack(np.interp(ti, time, varxp[:,0]))
varxp_inter = np.column_stack(varxp_inter)

# Plot X axis
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

ax.scatter(time, meanxp[:,0], marker='D', label="GP mean residual", color=[1.0,0.0,0.0], s=80)

ax.plot(ti, meanxp_inter[:,0], marker='*', linestyle='', label="Interpolation",
        color=[0.0,0.0,1.0], lw=1)
ax.plot(ti, (meanxp_inter[:,0]+3.0*varxp_inter[:,0]).flatten(), linestyle='--',
        color=[0.0,1.0,0.0], lw=1.0)
ax.plot(ti, (meanxp_inter[:,0]-3.0*varxp_inter[:,0]).flatten(), linestyle='--',
        color=[0.0,1.0,0.0], lw=1.0)


plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


# Plot Y axis
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(2)
ax = fig.add_subplot(111)

ax.scatter(time, meanxp[:,1], marker='D', label="GP mean residual", color=[1.0,0.0,0.0], s=80)

ax.plot(ti, meanxp_inter[:,1], marker='*', linestyle='', label="Interpolation",
        color=[0.0,0.0,1.0], lw=1)

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

# Plot Z axis
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(3)
ax = fig.add_subplot(111)

ax.scatter(time, meanxp[:,2], marker='D', label="GP mean residual", color=[1.0,0.0,0.0], s=80)

ax.plot(ti, meanxp_inter[:,2], marker='*', linestyle='', label="Interpolation",
        color=[0.0,0.0,1.0], lw=1)

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


####### REFERENCE ERROR ###########################
time = reference_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
ti = np.linspace(min(time), max(time), npts)

np.argmax(error, 0) # return the index pf the maximum indexes

# Interpolate Ground Truth Error 3-Dimensions
error_inter = np.column_stack((
                    np.interp(ti, time, error[:,0]),
                    np.interp(ti, time, error[:,1]),
                    np.interp(ti, time, error[:,2])
                    ))

# Plot X axis
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

ax.scatter(time, error[:,0], marker='D', label="Reference residual", color=[1.0,0.0,0.0], s=80)

ax.plot(ti, error_inter[:,0], marker='*', linestyle='', label="Interpolation",
        color=[0.0,1.0,0.0], lw=1)

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

# Plot Y axis
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(2)
ax = fig.add_subplot(111)

ax.scatter(time, error[:,1], marker='D', label="GP mean residual", color=[1.0,0.0,0.0], s=80)

ax.plot(ti, error_inter[:,1], marker='*', linestyle='', label="Interpolation",
        color=[0.0,1.0,0.0], lw=1)

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

# Plot Z axis
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(3)
ax = fig.add_subplot(111)

ax.scatter(time, error[:,2], marker='D', label="GP mean residual", color=[1.0,0.0,0.0], s=80)

ax.plot(ti, error_inter[:,2], marker='*', linestyle='', label="Interpolation",
        color=[0.0,1.0,0.0], lw=1)

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


