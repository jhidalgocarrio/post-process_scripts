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

path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141025-0005_odometry_comparison/'

# ################## #
# Planetary Lab Data #
# ################## #
path_odometry_file = path + 'pose_odo_position.0.data'

path_skid_file = path + 'pose_skid_position.0.data'

path_reference_file =  path + 'pose_ref_position.0.data'
#######################################

numbersamples=200

odoPos = data.ThreeData()
odoPos.readData(path_odometry_file, cov=True)
odoPos.eigenValues()

#Create a mean value to synchronize the trajectories
odotime = []
odoposx = []
odoposy = []
odoposz = []
for i in range(0, len(odoPos.getAxis(0)), numbersamples):
    odotime.append(mean(odoPos.t[0+i:numbersamples+i]))
    odoposx.append(mean(odoPos.getAxis(0)[0+i:numbersamples+i]))
    odoposy.append(mean(odoPos.getAxis(1)[0+i:numbersamples+i]))
    odoposz.append(mean(odoPos.getAxis(2)[0+i:numbersamples+i]))

odopos=[]
odopos.append(np.array(odoposx))
odopos.append(np.array(odoposy))
odopos.append(np.array(odoposz))

del(odoposx, odoposy, odoposz)

skidodoPos = data.ThreeData()
skidodoPos.readData(path_skid_file, cov=True)
skidodoPos.eigenValues()

#Create a mean value to synchronize the trajectories
skidtime = []
skidposx = []
skidposy = []
skidposz = []
for i in range(0, len(skidodoPos.getAxis(0)), numbersamples):
    skidtime.append(mean(skidodoPos.t[0+i:numbersamples+i]))
    skidposx.append(mean(skidodoPos.getAxis(0)[0+i:numbersamples+i]))
    skidposy.append(mean(skidodoPos.getAxis(1)[0+i:numbersamples+i]))
    skidposz.append(mean(skidodoPos.getAxis(2)[0+i:numbersamples+i]))


skidpos=[]
skidpos.append(np.array(skidposx))
skidpos.append(np.array(skidposy))
skidpos.append(np.array(skidposz))

del(skidposx, skidposy, skidposz)

refPos = data.ThreeData()
refPos.readData(path_reference_file, cov=False)
refPos.eigenValues()

#Create a mean value to synchronize the trajectories
reftime = []
refposx = []
refposy = []
refposz = []
for i in range(0, len(refPos.getAxis(0)), numbersamples):
    reftime.append(mean(refPos.t[0+i:numbersamples+i]))
    refposx.append(mean(refPos.getAxis(0)[0+i:numbersamples+i]))
    refposy.append(mean(refPos.getAxis(1)[0+i:numbersamples+i]))
    refposz.append(mean(refPos.getAxis(2)[0+i:numbersamples+i]))


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

