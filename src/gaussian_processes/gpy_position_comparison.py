#!/usr/bin/env python

#######################################
path_odometry_file = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/pose_odo_position.0.data'

#path_skid_file = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/pose_skid_position.0.data'

path_reference_file = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/pose_ref_position.0.data'

joints_position_file = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/joints_position.0.data'

joints_speed_file = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/joints_speed.0.data'

pose_imu_orientation_file =  '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/pose_imu_acceleration.0.data'
#######################################

import sys
sys.path.insert(0, './src/core')
import csv
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov
import joints as js
import GPy

#ExoTeR Odometry
odometry = data.ThreeData()
odometry.readData(path_odometry_file, cov=True)
odometry.eigenValues()

#Skid Odometry
#skid = data.ThreeData()
#skid.readData(path_skid_file, cov=True)
#skid.eigenValues()


#Vicon Pose
reference = data.ThreeData()
reference.readData(path_reference_file, cov=True)
reference.eigenValues()

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

# Read Gaussian process model
gp_m = data.open_object('./data/gaussian_processes/gpy_uncertainty_model_1000_sparse_regression.out')

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
inertia = np.column_stack((imu_acc.getAxis(0), imu_acc.getAxis(1), imu_acc.getAxis(2)))

######################
# Orientation Inputs #
# Roll and Pitch angles in that order #
orient = np.column_stack((imu_orient.getEuler(2), imu_orient.getEuler(1)))

# GP Multidimensional vector
Xp = np.column_stack((joints, inertia, orient))

[mean, var] = gp_m.predict(Xp, full_cov=True)

#Position comparison versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = reference.time
xposition = reference.getAxis(0)
ax.plot(time, xposition, marker='D', linestyle='--', label="Reference trajectory", color=[0.5,0,0], alpha=0.5, lw=2)

variance = mean[:,0] * mean[:,0]
delta_t = np.mean(reference.delta)
variance = variance * (delta_t)
variance = np.cumsum(variance)

time = odometry.time
xposition = odometry.getAxis(0)
ax.plot(time, xposition, marker='o', linestyle='-.', label="Jacobian Odometry", color=[0.3,0.2,0.4], lw=2)
sigma = np.sqrt(variance[0:len(xposition)])
ax.fill(np.concatenate([time, time[::-1]]),
        np.concatenate([xposition - 1.9600 * sigma,
                       (xposition + 1.9600 * sigma)[::-1]]),
        alpha=.5, fc='b', ec='None', label='95% confidence interval')

ax.plot(time, (xposition - 1.9600 * sigma), color="black", alpha=1.0, lw=1.0)
ax.plot(time, (xposition + 1.9600 * sigma), color="black", alpha=1.0, lw=1.0)


plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Distance [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

#Position comparison X-Y plane
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

xposition = reference.getAxis(0)[0::50]
yposition = reference.getAxis(1)[0::50]
ax.plot(xposition, yposition, marker='D', linestyle='--', label="Vicon Reference", color=[0.5,0,0], alpha=0.5, lw=2)
ax.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
ax.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
ax.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-40, -40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))
ax.annotate(r'End', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(-40, +40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

xposition = reference.getAxis(0)[0::100]
yposition = reference.getAxis(1)[0::100]
xycov = reference.getCov(1)[0::100]
for i in range(0, len(xycov)):
    cov.plot_cov_ellipse(xycov[i], pos=[xposition[i], yposition[i]], nstd=3,
                    linewidth=2, alpha=0.2, facecolor=[0.4,0,0.4], edgecolor='black')



plt.rc('text', usetex=False)# activate latex text rendering
xposition = odometry.getAxis(0)[0::50]
yposition = odometry.getAxis(1)[0::50]
ax.plot(xposition, yposition, marker='o', linestyle='-.', label="Jacobian Odometry", color=[0.3,0.2,0.4], lw=2)

delta_t = np.mean(reference.delta)
xvariance = mean[:,0] * mean[:,0]
xvariance = variance * (delta_t)
xvariance = np.cumsum(xvariance)
yvariance = mean[:,1] * mean[:,1]
yvariance = variance * (delta_t)
yvariance = np.cumsum(yvariance)
xsigma = np.sqrt(xvariance[0:len(xposition)])
ysigma = np.sqrt(yvariance[0:len(yposition)])

ax.fill(np.concatenate([xposition - 1.9600 * xsigma,
                       (xposition + 1.9600 * xsigma)[::-1]]),
        np.concatenate([yposition - 1.9600 * ysigma,
                       (yposition + 1.9600 * ysigma)[::-1]]),
        alpha=.5, fc='b', ec='None', label='95% confidence interval')

ax.plot(time, (xposition - 1.9600 * sigma), color="black", alpha=1.0, lw=1.0)
ax.plot(time, (xposition + 1.9600 * sigma), color="black", alpha=1.0, lw=1.0)



#xposition = odometry.getAxis(0)[0::50]# reduce number of points
#yposition = odometry.getAxis(1)[0::50]
#xycov = odometry.getCov(1)[0::10]
#for i in range(0, len(xycov)):
#    cov.plot_cov_ellipse(xycov[i], pos=[xposition[i], yposition[i]], nstd=3,
#                    linewidth=2, alpha=0.5, facecolor='green', edgecolor='black')

#xposition = skid.getAxis(0)[0::50]
#yposition = skid.getAxis(1)[0::50]
#ax.plot(xposition, yposition, marker='x', linestyle='--', label="Planar Odometry", color=[0,0.5,1], lw=2)


plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

