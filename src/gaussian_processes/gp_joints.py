#!/usr/bin/env python

#######################################
joints_position_file = '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/joints_position.0.data'

joints_speed_file = '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/joints_speed.0.data'

joints_effort_file = '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/joints_effort.0.data'

pose_ref_position_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_ref_position.0.data'

pose_ref_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_ref_velocity.0.data'

pose_odo_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_odo_velocity.0.data'

pose_imu_orientation_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_imu_acceleration.0.data'
#######################################
import sys
sys.path.insert(0, './src/core')
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov
import joints as js

from scipy.signal import filter_design as fd
import scipy.signal as sig


# Robot Joints Position and Speed
names = "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"

robot_joints = js.Joints(names)
robot_joints.readData(joints_position_file, joints_speed_file)

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

# IMU angular velocity
imu_angular = data.ThreeData()
imu_angular.readData(pose_imu_angular_velocity_file, cov=False)
imu_angular.eigenValues()

# IMU acceleration
imu_acc = data.ThreeData()
imu_acc.readData(pose_imu_acceleration_file, cov=False)
imu_acc.eigenValues()

####################################################


flsteer = robot_joints.getJoint("fl_steer")
pflsteer = robot_joints.getPosition("fl_steer")
sflsteer = robot_joints.getSpeed("fl_steer")

sfltranslation = robot_joints.getSpeed("fl_translation")
sfrtranslation = robot_joints.getSpeed("fr_translation")
smltranslation = robot_joints.getSpeed("ml_translation")
smrtranslation = robot_joints.getSpeed("mr_translation")
srltranslation = robot_joints.getSpeed("rl_translation")
srrtranslation = robot_joints.getSpeed("rr_translation")

# Plot FL Steer versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax1 = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = robot_joints.time
ax1.plot(time, pflsteer, marker='o', linestyle='-.', label="FL Steer position", color=[0.3,0.2,0.4], lw=2)
ax1.set_xlabel(r'T [$s$]', fontsize=35, fontweight='bold')
ax1.set_ylabel(r'Position [$rad$]', fontsize=35, fontweight='bold', color='b')
for tl in ax1.get_yticklabels():
        tl.set_color('b')

ax2 = ax1.twinx()
ax2.plot(time, sflsteer, marker='x', linestyle='--', label="FL Steer speed", color=[0.3,0.2,0.0], lw=2)
ax2.set_ylabel(r'Speed [$rad/s$]', fontsize=35, fontweight='bold', color='r')
for tl in ax2.get_yticklabels():
        tl.set_color('r')

plt.grid(True)
ax1.legend(loc=1, prop={'size':30})
ax2.legend(loc=1, prop={'size':30})
plt.show(block=False)


# Plot FL Steer versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = robot_joints.time
ax.plot(time, sfltranslation, marker='x', linestyle='--', label="FL Translation speed", color=[0.3,0.2,0.0], lw=2)

plt.xlabel(r'T [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Speed [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

# Robot Velocity versus time
plt.figure(1)
values = odometry_velocity.getAxis(0)
plt.plot(odometry_velocity.time, values,
        marker='.', label="Odometry Velocity X-axis", color=[1,0,0], lw=2)
plt.plot(odometry_velocity.time, odometry_velocity.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry_velocity.time, odometry_velocity.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)
values=reference_velocity.getAxis(0)
plt.plot(reference_velocity.time, values,
        marker='D', label="Ground Truth X-axis", color=[0,0.5,0.5], alpha=0.5, lw=2)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plt.figure(1)
values = odometry_velocity.getAxis(1)
plt.plot(odometry_velocity.time, values,
        marker='.', label="Odometry Velocity Y-axis", color=[1,0,0], lw=2)
plt.plot(odometry_velocity.time, odometry_velocity.getStdMax(1, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry_velocity.time, odometry_velocity.getStdMin(1, 3) , color=[0,0,0], linestyle='--', lw=2)
values=reference_velocity.getAxis(1)
plt.plot(reference_velocity.time, values,
        marker='D', label="Ground Truth Y-axis", color=[0,0.5,0.5], alpha=0.5, lw=2)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


# Plot Joint versus Velocity
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax1 = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
minindex = int(min(len(reference_velocity.t), len(robot_joints.t)))
velocity = reference_velocity.getAxis(0)[0:minindex]
mycolor = [0.0,0.2,0.4]
ax1.plot(pflsteer[0:minindex], velocity, marker='o', linestyle='-.', label="FL Translation speed", color=mycolor, lw=2)
ax1.set_xlabel(r'Joint Speed [$m/s$]', fontsize=35, fontweight='bold')
ax1.set_ylabel(r'Robot Velocity [$m/s$]', fontsize=35, fontweight='bold', color=mycolor)
for tl in ax1.get_yticklabels():
        tl.set_color(mycolor)

plt.grid(True)
ax1.legend(loc=1, prop={'size':30})
plt.show(block=False)


# Robot Odometry Velocity versus time with error bars
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

values = odometry_velocity.getAxis(0)
ax.errorbar(odometry_velocity.time, values, yerr = odometry_velocity.getStd(0, 3), fmt=None,ecolor='r',zorder=1)
ax.plot(odometry_velocity.time, values, 'kx',mew=1.5, label="Odometry Velocity X-axis")
plt.plot(odometry_velocity.time, odometry_velocity.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(odometry_velocity.time, odometry_velocity.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

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

Filters = {'butter' : ()}
Filters['butter'] = sig.butter(filter_order, [lowcup, highcup], btype='lowpass')

velocityfilter = {'reference_velocity_x' : (), 'reference_velocity_y' : (), 'reference_velocity_z' : ()}
velocityfilter['reference_velocity_x'] = sig.lfilter(Filters['butter'][0], Filters['butter'][1], reference_velocity.getAxis(0))
velocityfilter['reference_velocity_y'] = sig.lfilter(Filters['butter'][0], Filters['butter'][1], reference_velocity.getAxis(1))
velocityfilter['reference_velocity_z'] = sig.lfilter(Filters['butter'][0], Filters['butter'][1], reference_velocity.getAxis(2))

# Plotting the filtered values
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = reference_velocity.time
xvelocity = reference_velocity.getAxis(0)
ax.plot(time, xvelocity,  linestyle='-.', label="Raw Velocity", color=[0.0,0.2,0.4], lw=2)
time = reference_velocity.time
xvelocity = velocityfilter['reference_velocity_x']
ax.plot(time, xvelocity, linestyle='--', label="Filtered Velocity", color=[0.0,0.6,0], alpha=0.5, lw=4)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

# Robot Reference Velocity and Joints Translation Speed versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
velocity = reference_velocity.getAxis(0)
mycolor = [1.0,1.0,0.4]
ax.plot(reference_velocity.time, velocity, marker='o', linestyle='-.', label="Robot", color=mycolor, lw=2)
mycolor = [0.0,0.0,0.2]
ax.plot(robot_joints.time, sfltranslation, linestyle='-.', label="FL Translation speed", color=mycolor, lw=2)
mycolor = [0.0,0.0,0.4]
ax.plot(robot_joints.time, sfrtranslation, linestyle='-.', label="FL Translation speed", color=mycolor, lw=2)
mycolor = [0.0,0.0,0.6]
ax.plot(robot_joints.time, smltranslation, linestyle='-.', label="FL Translation speed", color=mycolor, lw=2)
mycolor = [0.0,0.0,0.8]
ax.plot(robot_joints.time, smrtranslation, linestyle='-.', label="FL Translation speed", color=mycolor, lw=2)
mycolor = [0.0,0.0,1.0]
ax.plot(robot_joints.time, srltranslation, linestyle='-.', label="FL Translation speed", color=mycolor, lw=2)
mycolor = [0.0,0.2,0.2]
ax.plot(robot_joints.time, srrtranslation, linestyle='-.', label="FL Translation speed", color=mycolor, lw=2)
ax.set_xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
ax.set_ylabel(r'Robot Velocity [$m/s$]', fontsize=35, fontweight='bold')

plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


# Robot IMU acceleration versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax1 = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
velocity = reference_velocity.getAxis(0)
mycolor = [1.0,1.0,0.4]
ax1.plot(reference_velocity.time, velocity, marker='o', linestyle='-.', label="Robot", color=mycolor, lw=2)
ax1.set_xlabel(r'T [$s$]', fontsize=35, fontweight='bold')
ax1.set_ylabel(r'Velocity [$m/s$]', fontsize=35, fontweight='bold', color='b')
for tl in ax1.get_yticklabels():
        tl.set_color('b')


ax2 = ax1.twinx()
accx = imu_acc.getAxis(0)
ax2.plot(imu_acc.time, accx, linestyle='-.', label="Acc X", color='r', lw=2)
accy = imu_acc.getAxis(1)
ax2.plot(imu_acc.time, accy, linestyle='-.', label="Acc Y", color='g', lw=2)
accz = imu_acc.getAxis(2)
ax2.plot(imu_acc.time, accz, linestyle='-.', label="Acc Z", color='b', lw=2)
ax2.set_ylabel(r'Acceleration [$m/s^2$]', fontsize=35, fontweight='bold', color='r')
for tl in ax2.get_yticklabels():
        tl.set_color('r')


plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


######################
## GAUSSIAN PROCESS ##
######################

from sklearn.gaussian_process import GaussianProcess

np.random.seed(1)


# Observation and noise
X = np.array([])
y = np.array([])
k = []

for i in range(0, 20):
    k = np.append(k, np.random.randint(0, len(robot_joints.time)))

k = sorted(k)

for i in range(0, len(k)):
    X = np.append(X, float(robot_joints.time[int(k[i])]))
    y = np.append(y, float(sfltranslation[int(k[i])]))


y = y.ravel()
X = np.atleast_2d(X).T
dy = 0.001 * np.random.random(y.shape)

# Instanciate a Gaussian Process model
gp = GaussianProcess(corr='squared_exponential', theta0=1e-1,
                     thetaL=1e-3, thetaU=0.4,
                     nugget=(dy ) ** 2,
                     random_start=100)


# Fit to data using Maximum Likelihood Estimation of the parameters
gp.fit(X, y)

# Make the prediction on the meshed x-axis (ask for MSE as well)
x = np.atleast_2d(robot_joints.time).T
y_pred, MSE = gp.predict(x, eval_MSE=True)
sigma = np.sqrt(MSE)

# Plot the function, the prediction and the 95% confidence interval based on
# the MSE
fig = plt.figure()
plt.plot(x, sfltranslation, 'r:', label=u'Joint Speed Trajectory')
plt.errorbar(X.ravel(), y, dy, fmt='r.', markersize=10, label=u'Observations')
plt.plot(x, y_pred, 'b-', label=u'Prediction')
plt.fill(np.concatenate([x, x[::-1]]),
        np.concatenate([y_pred - 1.9600 * sigma,
                       (y_pred + 1.9600 * sigma)[::-1]]),
        alpha=.5, fc='b', ec='None', label='95% confidence interval')
plt.xlabel('$x$')
plt.ylabel('$f(x)$')
plt.ylim(-10, 20)
plt.legend(loc='upper left')

pl.show(block=False)


######################
## GAUSSIAN PROCESS ##
######################

import GPy

# Observation and noise
X = np.array([])
Y = np.array([])
k = []

for i in range(0, 2000):
    k = np.append(k, np.random.randint(0, len(robot_joints.time)))

k = sorted(k)

for i in range(0, len(k)):
    X = np.append(X, float(robot_joints.time[int(k[i])]))
    Y = np.append(Y, float(sfltranslation[int(k[i])]))

X = np.atleast_2d(X).T
Y = np.atleast_2d(Y).T

#kernel = GPy.kern.RBF(input_dim=1, variance=1., lengthscale=1.)
kernel = GPy.kern.Matern52(input_dim=1, variance=1., lengthscale=1.)

m = GPy.models.GPRegression(X,Y,kernel)

print m

m.plot()

m.optimize()

m.plot()

# Plot the GP with respect to the real function
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

m.plot(ax=ax)
ax.plot(robot_joints.time, sfltranslation, linestyle='--', label="Wheel Velocity", color=[0.0,1.0,0.0], lw=2)
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

#########################
## 2D GAUSSIAN PROCESS ##
#########################

# sample inputs and outputs
X = np.random.uniform(-3.,3.,(50,2))
Y = np.sin(X[:,0:1]) * np.sin(X[:,1:2])+np.random.randn(50,1)*0.05

# define kernel
ker = GPy.kern.Matern52(2,ARD=True) + GPy.kern.White(2)

# create simple GP model
m = GPy.models.GPRegression(X,Y,ker)

# optimize and plot
m.optimize(max_f_eval = 1000)
m.plot()
print(m)

# Plot slides of the function
figure, axes = plt.subplots(3,1)
for ax, y in zip(axes, [-1, 0, 1.5]):
    m.plot(fixed_inputs=[(1,y)], ax=ax)
    ax.set_ylabel('y=%d'%y)
    plt.suptitle('horizontal slices through the function')

# Plot slides of the function (without all data displayed)
figure, axes = plt.subplots(3,1)
for ax, y in zip(axes, [-1, 0, 1.5]):
    m.plot(fixed_inputs=[(1,y)], ax=ax, which_data_rows=[])
    ax.set_ylabel('y=%d'%y)
    plt.suptitle('vertical slices through the function')


# Predict

size=80
xaxis=np.linspace(-8,8,size)
yaxis=np.linspace(-8,8,size)
xp=np.ndarray(0)
yp=np.ndarray(0)

for a in xaxis:
    xp=np.hstack([xp, [a]*size])
    yp=np.hstack([yp,yaxis])
Xp=np.vstack([xp,yp])
print Xp

[mean,cov]=m.predict(Xp.T, full_cov=False)

mean_grid= np.hstack(split(mean,size))
cov_grid= np.hstack(split(cov,size))

# Plot the mean function
CS = plt.contour(xaxis,yaxis,mean_grid,18)
cbar = plt.colorbar(CS)
cbar.ax.set_ylabel('mean value')
# Add the contour line levels to the colorbar
cbar.add_lines(CS)

# Plot the covariance function
plt.figure()
CS = plt.contour(xaxis,yaxis,cov_grid,18)
# Add the contour line levels to the colorbar
cbar = plt.colorbar(CS)
cbar.ax.set_ylabel('covariance value')


