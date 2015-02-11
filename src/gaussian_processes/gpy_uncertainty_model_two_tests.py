#!/usr/bin/env python

#######################################
joints_position_file = '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/joints_position.0.data'

joints_speed_file = '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/joints_speed.0.data'

pose_ref_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_ref_velocity.0.data'

pose_odo_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_odo_velocity.0.data'

pose_imu_orientation_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_imu_acceleration.0.data'
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

from scipy import integrate
from scipy.signal import filter_design as fd
import scipy.signal as sig

# Reference Robot Velocity
reference_velocity = data.ThreeData()
reference_velocity.readData(pose_ref_velocity_file, cov=True)
reference_velocity.eigenValues()

# Odometry Robot Velocity
odometry_velocity = data.ThreeData()
odometry_velocity.readData(pose_odo_velocity_file, cov=True)
odometry_velocity.eigenValues()

# IMU orientation
imu_orient = data.QuaternionData()
imu_orient.readData(pose_imu_orientation_file, cov=True)
imu_orient.eigenValues()

# IMU acceleration
imu_acc = data.ThreeData()
imu_acc.readData(pose_imu_acceleration_file, cov=False)
imu_acc.eigenValues()

# IMU Angular Velocity
imu_gyro = data.ThreeData()
imu_gyro.readData(pose_imu_angular_velocity_file, cov=False)
imu_gyro.eigenValues()

# Robot Joints Position and Speed
names = "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"

robot_joints = js.Joints(names)
robot_joints.readData(joints_position_file, joints_speed_file)

###################
### IIR FILTER  ###
###################

# Sample rate in Hz
delta_t = mean(reference_velocity.delta)
sample_rate = 1.0/delta_t

# The Nyquist rate of the signal.
nyq_rate = 0.5 * sample_rate

# The cutoff frequency of the filter (in Hz)
low_cut_hz = 2
high_cut_hz = 2

# Length of the filter (number of coefficients, i.e. the filter order + 1)
filter_order = 8

# Specification for our filter
lowcup = low_cut_hz/ nyq_rate
highcup = high_cut_hz/ nyq_rate

filters = {'butter' : ()}
filters['butter'] = sig.butter(filter_order, [lowcup, highcup], btype='lowpass')

#########################
## FIRST INPUT TEST    ##
#########################

np.random.seed(1)

#################
# Joints Inputs #
joints_first = np.column_stack((
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
                        robot_joints.getPosition("left_passive"),
                        robot_joints.getPosition("right_passive"),
                        robot_joints.getPosition("rear_passive")
                        ))
##################
# Inertia Inputs #
#inertia = np.column_stack((imu_acc.getAxis(0), imu_acc.getAxis(1), imu_acc.getAxis(2)))
inertia_first = np.column_stack((imu_acc.getAxis(0), imu_acc.getAxis(1), imu_acc.getAxis(2),
                           imu_gyro.getAxis(0), imu_gyro.getAxis(1), imu_gyro.getAxis(2)))

######################
# Orientation Inputs #
# Roll and Pitch angles in that order #
orient_first = np.column_stack((imu_orient.getEuler(2), imu_orient.getEuler(1)))

###########################
# Create Reference Output #
reference_first  = sig.lfilter(filters['butter'][0], filters['butter'][1], np.row_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2))))
reference_first = np.column_stack(reference_first)

###########################
# Create Odometry Output #
odometry_first  = sig.lfilter(filters['butter'][0], filters['butter'][1], np.row_stack((odometry_velocity.getAxis(0), odometry_velocity.getAxis(1), odometry_velocity.getAxis(2))))
odometry_first = np.column_stack(odometry_first)

######################
## LOAD SECOND TEST ##
######################
joints_position_file = '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141025-0005/joints_position.0.data'

joints_speed_file = '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141025-0005/joints_speed.0.data'

pose_ref_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141025-0005/pose_ref_velocity.0.data'

pose_odo_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141025-0005/pose_odo_velocity.0.data'

pose_imu_orientation_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141025-0005/pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141025-0005/pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141025-0005/pose_imu_acceleration.0.data'
#######################################

# Reference Robot Velocity
reference_velocity = data.ThreeData()
reference_velocity.readData(pose_ref_velocity_file, cov=True)
reference_velocity.eigenValues()

# Odometry Robot Velocity
odometry_velocity = data.ThreeData()
odometry_velocity.readData(pose_odo_velocity_file, cov=True)
odometry_velocity.eigenValues()

# IMU orientation
imu_orient = data.QuaternionData()
imu_orient.readData(pose_imu_orientation_file, cov=True)
imu_orient.eigenValues()

# IMU acceleration
imu_acc = data.ThreeData()
imu_acc.readData(pose_imu_acceleration_file, cov=False)
imu_acc.eigenValues()

# IMU Angular Velocity
imu_gyro = data.ThreeData()
imu_gyro.readData(pose_imu_angular_velocity_file, cov=False)
imu_gyro.eigenValues()

# Robot Joints Position and Speed
names = "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"

robot_joints = js.Joints(names)
robot_joints.readData(joints_position_file, joints_speed_file)

#######################################
# Form the New Inputs to predict
# Joints Inputs #
joints_second = np.column_stack((
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
                        robot_joints.getPosition("left_passive"),
                        robot_joints.getPosition("right_passive"),
                        robot_joints.getPosition("rear_passive")
                        ))
##################
# Inertia Inputs #
inertia_second = np.column_stack((imu_acc.getAxis(0), imu_acc.getAxis(1), imu_acc.getAxis(2),
                           imu_gyro.getAxis(0), imu_gyro.getAxis(1), imu_gyro.getAxis(2)))

######################
# Orientation Inputs #
# Roll and Pitch angles in that order #
orient_second = np.column_stack((imu_orient.getEuler(2), imu_orient.getEuler(1)))

###########################
# Create Reference Output #
reference_second  = sig.lfilter(filters['butter'][0], filters['butter'][1], np.row_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2))))
reference_second = np.column_stack(reference_second)


###########################
# Create Odometry Output #
odometry_second  = sig.lfilter(filters['butter'][0], filters['butter'][1], np.row_stack((odometry_velocity.getAxis(0), odometry_velocity.getAxis(1), odometry_velocity.getAxis(2))))
odometry_second = np.column_stack(odometry_second)

#################
## CONCATENATE ##
#################
joints = np.concatenate((joints_first, joints_second))
del(joints_first, joints_second)

inertia = np.concatenate((inertia_first, inertia_second))
del(inertia_first, inertia_second)

orient = np.concatenate((orient_first, orient_second))
del(orient_first, orient_second)

reference = np.concatenate((reference_first, reference_second))
del(reference_first, reference_second)

odometry = np.concatenate((odometry_first, odometry_second))
del(odometry_first, odometry_second)

####################
## ERROR VELOCITY ##
####################

length = min(reference.shape[0], odometry.shape[0])

error = abs(reference[0:length, :]) - abs(odometry[0:length, :])

###################################################################
# Create the random vector for sub sampling the whole data values #
percentage = 2.0
dimension = length

randomvector = np.array([])
for i in range(0, int(percentage * dimension)):
    randomvector = np.append(randomvector, np.random.randint(0, dimension))

randomvector = sorted(randomvector)
randomvector = np.unique(randomvector)
vectoridx = np.setdiff1d(xrange(dimension), randomvector)

print('Vector length for GP Data is: ', len(vectoridx))

vectoridx = np.row_stack((vectoridx))

# Sub sampling the joints inputs #
joints = np.delete(joints, randomvector, 0)
joints = joints.astype('float32')

# Sub sampling the inertia #
inertia = np.delete(inertia, randomvector, 0)
inertia = inertia.astype('float32')

# Sub sampling the orientation #
orient = np.delete(orient, randomvector, 0)
orient = orient.astype('float32')

# GP Multidimensional vector
#X = np.column_stack((joints[:,0:2])) #Only two joints
X = np.column_stack((joints, inertia, orient))
#X = np.column_stack((joints, inertia)) #No orientation
#X = np.column_stack((X))

###############
## GP OUTPUT ##
###############

reference = np.delete(reference, randomvector, 0)
reference = reference.astype('float32')

# GP Multidimensional vector
#Y =  np.column_stack((reference[:,0])) #Only one reference
Y =  np.column_stack((reference))
Y = np.column_stack(Y)

# define kernel
# The flag ARD=True in the definition of the Matern kernel specifies that we
# want one lengthscale parameter per dimension (ie the GP is not isotropic).
#ker = GPy.kern.RBF(input_dim = X.shape[1], ARD=True)
ker_rbf = GPy.kern.RBF(input_dim = X.shape[1], ARD=False)
ker_white = GPy.kern.White(input_dim = X.shape[1])
ker = ker_white + ker_rbf

# create simple GP model
m = GPy.models.GPRegression(X, Y, kernel=ker)

print m
# optimize and plot
m.optimize('bfgs', messages=True, max_f_eval=1000)
print(m)

##################
## PLOT VALUES ##
##################


#######################
## LOAD ANOTHER TEST ##
#######################
joints_position_file = '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141024-2317/joints_position.0.data'

joints_speed_file = '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141024-2317/joints_speed.0.data'

pose_ref_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141024-2317/pose_ref_velocity.0.data'

pose_odo_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141024-2317/pose_odo_velocity.0.data'

pose_imu_orientation_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141024-2317/pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141024-2317/pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141024-2317/pose_imu_acceleration.0.data'
#######################################

# Reference Robot Velocity
reference_velocity = data.ThreeData()
reference_velocity.readData(pose_ref_velocity_file, cov=True)
reference_velocity.eigenValues()

# Odometry Robot Velocity
odometry_velocity = data.ThreeData()
odometry_velocity.readData(pose_odo_velocity_file, cov=True)
odometry_velocity.eigenValues()

# IMU orientation
imu_orient = data.QuaternionData()
imu_orient.readData(pose_imu_orientation_file, cov=True)
imu_orient.eigenValues()

# IMU acceleration
imu_acc = data.ThreeData()
imu_acc.readData(pose_imu_acceleration_file, cov=False)
imu_acc.eigenValues()

# IMU Angular Velocity
imu_gyro = data.ThreeData()
imu_gyro.readData(pose_imu_angular_velocity_file, cov=False)
imu_gyro.eigenValues()

# Robot Joints Position and Speed
names = "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"

robot_joints = js.Joints(names)
robot_joints.readData(joints_position_file, joints_speed_file)

#######################################
# Form the New Inputs to predict
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

###################################################################
# Create the random vector for sub sampling the whole data values #
percentage = 2.0
dimension = min (len(odometry_velocity.time), len(reference_velocity.time))

randomvector = np.array([])
for i in range(0, int(percentage * dimension)):
    randomvector = np.append(randomvector, np.random.randint(0, dimension))

randomvector = sorted(randomvector)
randomvector = np.unique(randomvector)
vectoridx = np.setdiff1d(xrange(dimension), randomvector)

print('Vector length for GP Data is: ', len(vectoridx))

vectoridx = np.row_stack((vectoridx))

# Sub sampling the joints inputs #
joints = np.delete(joints, randomvector, 0)
joints = joints.astype('float32')

# Sub sampling the inertia #
inertia = np.delete(inertia, randomvector, 0)
inertia = inertia.astype('float32')

# Sub sampling the orientation #
orient = np.delete(orient, randomvector, 0)
orient = orient.astype('float32')

# GP Multidimensional vector
Xp = np.column_stack((joints, inertia, orient))

##################
#Reference and GP Velocity comparison X-Time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
xvelocity = reference_velocity.getAxis(0)
ax.plot(reference_velocity.time, xvelocity, linestyle='-.', label="Reference Velocity", color=[0.3,0.2,0.4], lw=2)

#[mean1, var1] = m.predict(Xp, full_cov=True)
variance = mean1[:,0] * mean1[:,0]

xtime = np.array(odometry_velocity.time)
xtime = np.delete(xtime, randomvector, 0)
xvelocity = odometry_velocity.getAxis(0)
xvelocity = np.delete(xvelocity, randomvector, 0)
sigma = np.sqrt(variance[0:len(xvelocity)])

ax.plot(xtime, xvelocity, linestyle='-.', label="Odometry Velocity", color=[0.0,0.0,1.0], lw=3)
ax.fill(np.concatenate([xtime, xtime[::-1]]),
        np.concatenate([xvelocity - 1.9600 * sigma,
                       (xvelocity + 1.9600 * sigma)[::-1]]),
        alpha=.5, fc='b', ec='None', label='95% confidence interval')

ax.plot(xtime, (xvelocity - 1.9600 * sigma), color="black", alpha=1.0, lw=1.0)
ax.plot(xtime, (xvelocity + 1.9600 * sigma), color="black", alpha=1.0, lw=1.0)

plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Velocity [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)



