#!/usr/bin/env python

# ################## #
# Decos Terrain Data #
# ################## #

path = '/home/javi/exoter/development/data/20140911_decos_field/20140911-1805_odometry_comparison_bis/'
#######################################
odometry_file = path + 'pose_odo_position.reaction_forces.0.data'

skid_file = path + 'pose_skid_position.0.data'

reference_file = path + 'pose_ref_position.0.data'
#######################################
delta_reference_file =  path + 'delta_pose_odo_position.0.data'
#######################################
pose_odo_orient_file = path + "pose_odo_orientation.reaction_forces.0.data"

pose_ref_orient_file = path + "pose_ref_orientation.0.data"
#######################################


import sys
sys.path.insert(0, './src/core')
import datadisplay as data
from random import gauss
import csv, scipy
import math
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
from math import sqrt

#ExoTeR Odometry
threed_odometry = data.ThreeData()
threed_odometry.readData(odometry_file, cov=True)

# Read the odometry orientation information
odometry_orient = data.QuaternionData()
odometry_orient.readData(pose_odo_orient_file, cov=True)

#Skid Odometry
skid_odometry = data.ThreeData()
skid_odometry.readData(skid_file, cov=True)

#GNSS Pose
reference = data.ThreeData()
reference.readData(reference_file, cov=True)

#GNSS Delta Pose
delta_reference = data.ThreeData()
delta_reference.readData(delta_reference_file, cov=True)

# Read the reference orientation information
reference_orient = data.QuaternionData()
reference_orient.readData(pose_ref_orient_file, cov=True)


########################
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference.cov[:,0,0]))
temindex = np.asarray(temindex)

reference.delete(temindex)
threed_odometry.delete(temindex)
skid_odometry.delete(temindex)

#######################################
### COMPUTE COVARIANCE EIGENVALUES  ###
#######################################
reference.eigenValues()
threed_odometry.eigenValues()
skid_odometry.eigenValues()

###########################
##   Distance traveled   ##
###########################
# No use the z axis when using
# reference because GPS is really bad in Z
# direction
position = np.column_stack((delta_reference.getAxis(0),
    delta_reference.getAxis(1), delta_reference.getAxis(1)))

norm_delta_position = []
norm_delta_position = [np.linalg.norm(x) for x in position ]
distance_traveled = np.nansum(norm_delta_position)

#################################################
# Take the misalignment between both orientations
#################################################
#misalignment = ~odometry_orient.data[1000] * reference_orient.data[1000] #~ is invert operator
misalignment = quat.quaternion(np.array([1.0,0.0,0.0,0.0]))

#################################################
# Align odometry with the staring orientation of 
# the GNSS information 
#################################################
odopos = np.column_stack((threed_odometry.getAxis(0), threed_odometry.getAxis(1),
        threed_odometry.getAxis(2)))
odopos[:] = [(misalignment * x * misalignment.conj())[1:4] for x in odopos ]

###################
###    MEAN     ###
###################
numbersamples=200
#Create a mean value to synchronize the trajectories
odotime = []
odoposx = []
odoposy = []
odoposz = []
for i in range(0, len(threed_odometry.getAxis(0)), numbersamples):
    odotime.append(mean(threed_odometry.t[0+i:numbersamples+i]))
    odoposx.append(mean(odopos[0+i:numbersamples+i,0]))
    odoposy.append(mean(odopos[0+i:numbersamples+i,1]))
    odoposz.append(mean(odopos[0+i:numbersamples+i,2]))

odopos=[]
odopos.append(np.array(odoposx))
odopos.append(np.array(odoposy))
odopos.append(np.array(odoposz))

del(odoposx, odoposy, odoposz)

#Create a mean value to synchronize the trajectories
skidtime = []
skidposx = []
skidposy = []
skidposz = []
for i in range(0, len(skid_odometry.getAxis(0)), numbersamples):
    skidtime.append(mean(skid_odometry.t[0+i:numbersamples+i]))
    skidposx.append(mean(skid_odometry.getAxis(0)[0+i:numbersamples+i]))
    skidposy.append(mean(skid_odometry.getAxis(1)[0+i:numbersamples+i]))
    skidposz.append(mean(skid_odometry.getAxis(2)[0+i:numbersamples+i]))


skidpos=[]
skidpos.append(np.array(skidposx))
skidpos.append(np.array(skidposy))
skidpos.append(np.array(skidposz))

del(skidposx, skidposy, skidposz)

#Create a mean value to synchronize the trajectories
reftime = []
refposx = []
refposy = []
refposz = []
for i in range(0, len(reference.getAxis(0)), numbersamples):
    reftime.append(mean(reference.t[0+i:numbersamples+i]))
    refposx.append(mean(reference.getAxis(0)[0+i:numbersamples+i]))
    refposy.append(mean(reference.getAxis(1)[0+i:numbersamples+i]))
    refposz.append(mean(reference.getAxis(2)[0+i:numbersamples+i]))


refpos=[]
refpos.append(np.array(refposx))
refpos.append(np.array(refposy))
refpos.append(np.array(refposz))

del(refposx, refposy, refposz)

# Plot to verify the data to analyze
plot(odopos[0], odopos[1], color='blue')
plot(skidpos[0], skidpos[1], color='red')
plot(refpos[0], refpos[1], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(odopos[0], odopos[2], color='blue')
plot(skidpos[0], skidpos[2], color='red')
plot(refpos[0], refpos[2], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

#Plot over  time
plot(odotime, odopos[0], color='blue',  marker='x')
plot(skidtime, skidpos[0], color='red', marker='x')
plot(reftime, refpos[0], color='black', marker='x')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in X [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(odotime, odopos[1])
plot(skidtime, skidpos[1], color='red')
plot(reftime, refpos[1], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(odotime, odopos[2])
plot(skidtime, skidpos[2], color='red')
plot(reftime, refpos[2], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

# Plot error over time
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odoerrorx = np.absolute(odopos[0][0:datasize[0]] - refpos[0][0+3:datasize[0]+3])
odoerrory = np.absolute(odopos[1][0:datasize[0]] - refpos[1][0+3:datasize[0]+3])
odoerrorz = np.absolute(odopos[2][0:datasize[0]] - refpos[2][0+3:datasize[0]+3])

plot(odotime, odoerrorx, marker='o')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position Error in X [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


# RMSE for Full Odometry
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odormse = []
odormse.append(sqrt(((odopos[0][0:datasize[0]] - refpos[0][0:datasize[0]]) ** 2).mean()))
odormse.append(sqrt(((odopos[1][0:datasize[1]] - refpos[1][0:datasize[1]]) ** 2).mean()))
odormse.append(sqrt(((odopos[2][0:datasize[2]] - refpos[2][0:datasize[2]]) ** 2).mean()))

#Final error position for Full Odometry
odofinale = []
odofinale.append(np.absolute(odopos[0][len(odopos[0])-1] - refpos[0][len(refpos[0])-1]))
odofinale.append(np.absolute(odopos[1][len(odopos[2])-1] - refpos[1][len(refpos[1])-1]))
odofinale.append(np.absolute(odopos[2][len(odopos[2])-1] - refpos[2][len(refpos[2])-1]))

# Maximum error for Full Odometry
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odomaxe = []
odomaxe.append(np.absolute(odopos[0][0:datasize[0]] - refpos[0][0:datasize[0]]).max())
odomaxe.append(np.absolute(odopos[1][0:datasize[1]] - refpos[1][0:datasize[1]]).max())
odomaxe.append(np.absolute(odopos[2][0:datasize[2]] - refpos[2][0:datasize[2]]).max())

# Median error for Full Odometry
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odomediane = []
odomediane.append(np.median(np.absolute(odopos[0][0:datasize[0]] - refpos[0][0:datasize[0]])))
odomediane.append(np.median(np.absolute(odopos[1][0:datasize[1]] - refpos[1][0:datasize[1]])))
odomediane.append(np.median(np.absolute(odopos[2][0:datasize[2]] - refpos[2][0:datasize[2]])))

# RMSE for Skid Odometry
datasize=[]
datasize.append(min(len(skidpos[0]), len(refpos[0])))
datasize.append(min(len(skidpos[1]), len(refpos[1])))
datasize.append(min(len(skidpos[2]), len(refpos[2])))
skidrmse = []
skidrmse.append(sqrt(((skidpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]) ** 2).mean()))
skidrmse.append(sqrt(((skidpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]) ** 2).mean()))
skidrmse.append(sqrt(((skidpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]) ** 2).mean()))

#Final error position for Skid Odometry
skidfinale = []
skidfinale.append(np.absolute(skidpos[0][len(skidpos[0])-1] - refpos[0][len(refpos[0])-1]))
skidfinale.append(np.absolute(skidpos[1][len(skidpos[1])-1] - refpos[1][len(refpos[1])-1]))
skidfinale.append(np.absolute(skidpos[2][len(skidpos[2])-1] - refpos[2][len(refpos[2])-1]))

# Maximum error for Skid Odometry
datasize=[]
datasize.append(min(len(skidpos[0]), len(refpos[0])))
datasize.append(min(len(skidpos[1]), len(refpos[1])))
datasize.append(min(len(skidpos[2]), len(refpos[2])))
skidmaxe = []
skidmaxe.append(np.absolute(skidpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]).max())
skidmaxe.append(np.absolute(skidpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]).max())
skidmaxe.append(np.absolute(skidpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]).max())

# Median error for Skid Odometry
datasize=[]
datasize.append(min(len(skidpos[0]), len(refpos[0])))
datasize.append(min(len(skidpos[1]), len(refpos[1])))
datasize.append(min(len(skidpos[2]), len(refpos[2])))
skidmediane = []
skidmediane.append(np.median(np.absolute(skidpos[0][0:datasize[0]] - refpos[0][0:datasize[0]])))
skidmediane.append(np.median(np.absolute(skidpos[1][0:datasize[1]] - refpos[1][0:datasize[1]])))
skidmediane.append(np.median(np.absolute(skidpos[2][0:datasize[2]] - refpos[2][0:datasize[2]])))

# Print values
odormse
skidrmse

odomaxe
skidmaxe

odomediane
skidmediane

odofinale
skidfinale

