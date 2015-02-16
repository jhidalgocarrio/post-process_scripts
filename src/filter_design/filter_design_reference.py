#!/usr/bin/env python

#######################################
pose_ref_position_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_ref_position.0.data'

pose_ref_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_ref_velocity.0.data'
#######################################
# Also load Odometry info to compare the output
pose_odo_position_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_odo_position.0.data'

pose_odo_velocity_file =  '/home/javi/exoter/development/post-process_data/20141024_planetary_lab/20141027-2034/pose_odo_velocity.0.data'
#######################################

import sys
sys.path.insert(0, './src/core')
import quaternion as quat
import datadisplay as data
import csv, scipy, numpy, pylab
import scipy.fftpack
from pylab import *
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.signal import filter_design as fd
import scipy.signal as sig


# Reference Robot position
reference_position = data.ThreeData()
reference_position.readData(pose_ref_position_file, cov=True)
reference_position.eigenValues()

# Reference Robot Velocity
reference_velocity = data.ThreeData()
reference_velocity.readData(pose_ref_velocity_file, cov=True)
reference_velocity.eigenValues()


# Reference Robot position
odometry_position = data.ThreeData()
odometry_position.readData(pose_odo_position_file, cov=True)
odometry_position.eigenValues()

# Reference Robot Velocity
odometry_velocity = data.ThreeData()
odometry_velocity.readData(pose_odo_velocity_file, cov=True)
odometry_velocity.eigenValues()

#######################################
# Remove the initial values
#[b,a] = butter(5,.7);
#N = 50; % change this to suit your needs
#    yNew = filtfilt(b,a,[y(N:-1:1);y];
#yNew = yNew(N+1:end);

#Position comparison versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = odometry_position.time
xposition = odometry_position.getAxis(1)
ax.plot(time, xposition, marker='o', linestyle='-.', label="Jacobian Odometry", color=[0.3,0.2,0.4], lw=2)
time = reference_position.time
xposition = reference_position.getAxis(1)
ax.plot(time, xposition, marker='D', linestyle='--', label="Vicon Reference", color=[0.5,0,0], alpha=0.5, lw=2)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


#Velocity comparison versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = odometry_velocity.time
xvelocity = odometry_velocity.getAxis(1)
ax.plot(time, xvelocity, marker='o', linestyle='-.', label="Jacobian Odometry", color=[0.0,0.2,0.4], lw=2)
time = reference_velocity.time
xvelocity = reference_velocity.getAxis(1)
ax.plot(time, xvelocity, marker='D', linestyle='--', label="Vicon Reference", color=[0.0,0.6,0], alpha=0.5, lw=2)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)


delta_t = mean(reference_velocity.delta)
sample_rate = 1.0/delta_t

# The Nyquist rate of the signal.
nyq_rate = sample_rate / 2.

# The cutoff frequency of the filter (in Hz)
cutoff_hz = 0.3

# Length of the filter (number of coefficients, i.e. the filter order + 1)
numtaps = 8

###################
### IIR FILTER  ###
###################

# Specification for our filter
Wp = cutoff_hz/nyq_rate # Cutoff frequency
Ws = (cutoff_hz+1.5)/nyq_rate   # Stop frequency
Rp = 1     # Ripple in the passband maximum loss (gpass)
As = 42      # Min Attenuation in the stoppand (gstop)

Filters = {'ellip' : (), 'cheby2' : (), 'butter' : (), 'cheby1' : (),  'bessel' : ()}

# The ellip and cheby2 filter design
Filters['ellip'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='ellip')
Filters['cheby2'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='cheby2')
Filters['butter'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='butter')
Filters['cheby1'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='cheby1')

# The bessel max order of 8 for this cutoff, can't use
# iirdesign have to use iirfilter.
Filters['bessel'] = fd.iirfilter(8, Wp, btype='lowpass', ftype='bessel')


## Pass the signal though the filter
velocity = reference_velocity.getAxis(0)
velocityfilter = {'ellip' : (), 'cheby2' : (), 'butter' : (), 'cheby1' : (),  'bessel' : ()}
velocityfilter['ellip'] = sig.lfilter(Filters['ellip'][0], Filters['ellip'][1], velocity)
velocityfilter['cheby2'] = sig.lfilter(Filters['cheby2'][0], Filters['cheby2'][1], velocity)
velocityfilter['butter'] = sig.lfilter(Filters['butter'][0], Filters['butter'][1], velocity)
velocityfilter['cheby1'] = sig.lfilter(Filters['cheby1'][0], Filters['cheby1'][1], velocity)
velocityfilter['bessel'] = sig.lfilter(Filters['bessel'][0], Filters['bessel'][1], velocity)

#################
### GRAPHICS  ###
#################
plt.figure(4)
time = reference_velocity.time
pylab.plot(time, velocity, '-o', label="Velocity")
pylab.plot(time,velocityfilter['ellip'], '--', label="ellip filter", lw=2)
pylab.plot(time,velocityfilter['cheby2'], '--', label="cheby2 filter", lw=2)
pylab.plot(time,velocityfilter['butter'], '--', label="butter filter", lw=2)
pylab.plot(time,velocityfilter['cheby1'], '--', label="cheby1 filter", lw=2)
pylab.plot(time,velocityfilter['bessel'], '--', label="bessel filter", lw=2)
plt.legend(prop={'size':25})
grid(True)
plt.show(block=False)

#Velocity comparison versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = odometry_velocity.time
xvelocity = odometry_velocity.getAxis(0)
ax.plot(time, xvelocity, marker='o', linestyle='-.', label="Jacobian Odometry", color=[0.0,0.2,0.4], lw=2)
time = reference_velocity.time
xvelocity = velocityfilter['butter']
ax.plot(time, xvelocity, marker='D', linestyle='--', label="Vicon Reference", color=[0.0,0.6,0], alpha=0.5, lw=2)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

