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
low_cut_hz = 1
high_cut_hz = 1

# Length of the filter (number of coefficients, i.e. the filter order + 1)
filter_order = 8

# Specification for our filter
lowcup = low_cut_hz/ nyq_rate
highcup = high_cut_hz/ nyq_rate

filters = {'butter' : ()}
filters['butter'] = sig.butter(filter_order, [lowcup, highcup], btype='lowpass')

#########################
## 1D GAUSSIAN PROCESS ##
#########################

#########################
## 2D GAUSSIAN PROCESS ##
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
                        robot_joints.getPosition("left_passive"),
                        robot_joints.getPosition("right_passive"),
                        robot_joints.getPosition("rear_passive")
                        ))
##################
# Inertia Inputs #
#inertia = np.column_stack((imu_acc.getAxis(0), imu_acc.getAxis(1), imu_acc.getAxis(2)))
inertia = np.column_stack((imu_acc.getAxis(0), imu_acc.getAxis(1), imu_acc.getAxis(2),
                           imu_gyro.getAxis(0), imu_gyro.getAxis(1), imu_gyro.getAxis(2)))

######################
# Orientation Inputs #
# Roll and Pitch angles in that order #
orient = np.column_stack((imu_orient.getEuler(2), imu_orient.getEuler(1)))

###################################################################
# Create the random vector for sub sampling the whole data values #
percentage = 2.0
dimension = min (len(imu_acc.getAxis(0)), len(imu_acc.getAxis(1)))

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

########################
# Create the GP output #
reference  = sig.lfilter(filters['butter'][0], filters['butter'][1], np.row_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2))))

reference = np.column_stack(reference)
reference = np.delete(reference, randomvector, 0)
reference = reference.astype('float32')

# GP Multidimensional vector
#Y =  np.column_stack((reference[:,0])) #Only one reference
Y =  np.column_stack((reference))
Y = np.column_stack(Y)

# define kernel
# The flag ARD=True in the definition of the Matern kernel specifies that we
# want one lengthscale parameter per dimension (ie the GP is not isotropic).
#ker = GPy.kern.Matern52(X.shape[1], ARD=True)
ker_rbf = GPy.kern.RBF(input_dim = X.shape[1], ARD=False)
#ker_white = GPy.kern.White(input_dim = X.shape[1])
#ker = ker_white + ker_rbf
ker =  ker_rbf

# create simple GP model
#m = GPy.models.GPRegression(X, Y, kernel=ker)
m = GPy.models.GPHeteroscedasticRegression(X, Y, kernel=ker)
#Z = np.array(np.row_stack((vectoridx, vectoridx)))
#Z = Z.astype(float32)
#m = GPy.models.SparseGPRegression(X=X, Y=Y, kernel=ker, num_inducing=100)


print m
# optimize and plot
m.optimize('bfgs', messages=True, max_f_eval=1000)
print(m)

m.plot()

figure, axes = plt.subplots(3,1)
for ax, y in zip(axes, [-0.01, 0, 0.01]):
    m.plot(fixed_inputs=[(1,y)], ax=ax, which_data_rows=[])
    ax.set_ylabel('y=%d'%y)
    plt.suptitle('vertical slices through the function')


# Predict
size=200
xaxis=np.linspace(-0.15,0.15,size)
yaxis=np.linspace(-0.15,0.15,size)
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
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
cs = plt.contour(xaxis,yaxis,mean_grid,18)
cbar = plt.colorbar(cs)
cbar.ax.set_ylabel('mean value')
# Add the contour line levels to the colorbar
cbar.add_lines(cs)

# Plot the covariance function
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.figure()
CS = plt.contour(xaxis,yaxis,cov_grid,18)
# Add the contour line levels to the colorbar
cbar = plt.colorbar(CS)
cbar.ax.set_ylabel('covariance value')

# Predict one values
Xp1 = np.column_stack(np.array([0.02, 0.03]))
m.predict(Xp1, full_cov=True)


# Predict one values
Xp1 = np.column_stack(X[490,:])
[mean1, var1] = m.predict(Xp1, full_cov=True)

#########################
# 3D Plotting values    #
#########################
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
ax = fig.add_subplot(111, projection='3d')


xvalue = np.array(X[:,0])
yvalue = np.array(X[:,1])
zvalue = np.array(Y[:,0])
ax.plot(xvalue, yvalue, zvalue, marker='o', linestyle='-.', label="Reference", color=[0.3,0.2,0.4], lw=2)

ax.grid(True)
ax.legend(loc=2, prop={'size':30})
ax.set_xlabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
ax.set_ylabel(r'Y [$m/s$]', fontsize=35, fontweight='bold')
ax.set_zlabel(r'Z [$m/s$]', fontsize=35, fontweight='bold')
#ax.plot([-5.0, 70.0], [-5.0, 140.0], [-10, 10], linestyle='none') # One way to set axis limits instead set_xlimit

def update_position(e):
    x2, y2, _ = proj3d.proj_transform(xposition[0], yposition[0], zposition[0], ax.get_proj())
    start_label.xy = x2,y2
    start_label.update_positions(fig.canvas.renderer)
    x2, y2, _ = proj3d.proj_transform(xposition[len(xposition)-1], yposition[len(yposition)-1], zposition[len(zposition)-1], ax.get_proj())
    end_label.xy = x2,y2
    end_label.update_positions(fig.canvas.renderer)
    fig.canvas.draw()

fig.canvas.mpl_connect('button_release_event', update_position)
plt.show(block=False)

#######################################
## INTEGRATION OF PREDICTION OUTPUTS ##
#######################################

pose_odo_position_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_odo_position.0.data'

pose_ref_position_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_ref_position.0.data'

#######################################
# Reference Robot Position
odometry_position = data.ThreeData()
odometry_position.readData(pose_odo_position_file, cov=True)
odometry_position.eigenValues()

# Reference Robot Position
reference_position = data.ThreeData()
reference_position.readData(pose_ref_position_file, cov=True)
reference_position.eigenValues()


#Odometry Position comparison X-Time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
xposition = odometry_position.getAxis(0)
ax.plot(odometry_position.time, xposition, marker='o', linestyle='-.', label="Odometry", color=[0.3,0.2,0.4], lw=2)

xvelocity = odometry_velocity.getAxis(0)
xposition = integrate.cumtrapz(xvelocity, odometry_velocity.time, initial=0)
ax.plot(odometry_velocity.time, xposition, marker='o', linestyle='-.', label="CumSum Odometry", color=[0,0.5,1.0], lw=2)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

#Reference and GP Velocity comparison X-Time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
reference  = sig.lfilter(filters['butter'][0], filters['butter'][1], np.row_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2))))
xvelocity = reference[0,:]
ax.plot(reference_velocity.time, xvelocity, marker='o', linestyle='-.', label="Reference Velocity", color=[0.3,0.2,0.4], lw=2)

[mean1, var1] = m.predict(X, full_cov=True)
xvelocity = mean1[:,0]
xtime = np.array(reference_velocity.time)
xtime = np.delete(xtime, randomvector, 0)
ax.plot(xtime, xvelocity, marker='o', linestyle='-.', label="GP Velocity", color=[0,0.5,1.0], lw=2)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

#Reference and GP Velocity comparison Y-Time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
reference  = sig.lfilter(filters['butter'][0], filters['butter'][1], np.row_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2))))
yvelocity = reference[1,:]
ax.plot(reference_velocity.time, yvelocity, marker='o', linestyle='-.', label="Reference Velocity", color=[0.3,0.2,0.4], lw=2)

[mean1, var1] = m.predict(X, full_cov=True)
yvelocity = mean1[:,1]
ytime = np.array(reference_velocity.time)
ytime = np.delete(ytime, randomvector, 0)
ax.plot(ytime, yvelocity, marker='o', linestyle='-.', label="GP Velocity", color=[0,0.5,1.0], lw=2)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

#Reference and GP Velocity comparison Z-Time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
reference  = sig.lfilter(filters['butter'][0], filters['butter'][1], np.row_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2))))
zvelocity = reference[2,:]
ax.plot(reference_velocity.time, zvelocity, marker='o', linestyle='-.', label="Reference Velocity", color=[0.3,0.2,0.4], lw=2)

[mean1, var1] = m.predict(X, full_cov=True)
zvelocity = mean1[:,2]
ztime = np.array(reference_velocity.time)
ztime = np.delete(ztime, randomvector, 0)
ax.plot(ztime, zvelocity, marker='o', linestyle='-.', label="GP Velocity", color=[0,0.5,1.0], lw=2)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

#Odometry, Reference and GP Position comparison X-Y plane
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
xposition = odometry_position.getAxis(0)
yposition = odometry_position.getAxis(1)
ax.plot(xposition, yposition, marker='o', linestyle='-.', label="Odometry", color=[0.3,0.2,0.4], lw=2)

[mean1, var1] = m.predict(X, full_cov=True)
time = np.array(reference_velocity.time)
time = np.delete(time, randomvector, 0)
xposition = integrate.cumtrapz(mean1[:,0], time, initial=0)
yposition = integrate.cumtrapz(mean1[:,1], time, initial=0)
ax.plot(xposition, yposition, marker='x', linestyle='.-', label="CumSum GP", color=[0.0,0.0,0.4], lw=2)

xposition = reference_position.getAxis(0)
yposition = reference_position.getAxis(1)
ax.plot(xposition, yposition, marker='D', linestyle='--', label="Vicon Reference", color=[0.5,0,0], alpha=0.5, lw=2)
ax.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
ax.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
ax.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-40, -40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))
ax.annotate(r'End', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(-40, +40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

#######################
## LOAD ANOTHER TEST ##
#######################
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
dimension = min (len(imu_acc.getAxis(0)), len(imu_acc.getAxis(1)))

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
#Xp = np.column_stack((joints, inertia)) #No orientation


#Reference and GP Velocity comparison X-Time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
reference  = sig.lfilter(filters['butter'][0], filters['butter'][1], np.row_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2))))
xvelocity = reference[0,:]
ax.plot(reference_velocity.time, xvelocity, marker='o', linestyle='-.', label="Reference Velocity", color=[0.3,0.2,0.4], lw=2)

[mean1, var1] = m.predict(Xp, full_cov=True)
xvelocity = mean1[:,0]
xtime = np.array(reference_velocity.time)
xtime = np.delete(xtime, randomvector, 0)
ax.plot(xtime, xvelocity, marker='o', linestyle='-.', label="GP Velocity", color=[0,0.5,1.0], lw=2)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


