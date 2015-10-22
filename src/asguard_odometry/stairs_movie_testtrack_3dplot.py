import datadisplay as data
from random import gauss
import csv, scipy
import math
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat

# Odometry + Support polygon Pose
odoPos = data.ThreeData()
odoPos.readData('../data/20130415_motion_model_test_track/20131206-2159/data/odometry_position.0.data', cov=True)
odoPos.eigenValues()

# Odometry Pose
pureodoPos = data.ThreeData()
pureodoPos.readData('../data/20130415_motion_model_test_track/20131206-2243/data/odometry_position.0.data', cov=True)
pureodoPos.eigenValues()


#3D Plotting  values
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.set_title('DFKI Test Track - Asguard')

ax.grid(True)
ax.legend(prop={'size':20})
ax.set_xlabel(r'Position [$m$]')
ax.set_ylabel(r'Position [$m$]')
ax.set_zlabel(r'Elevation [$m$]')
ax.plot([-5.0, 60.0], [-10.0, 4.0], [-1.0, 5.0], linestyle='none') # One way to set axis limits instead set_xlimit
ax.azim = 0
ax.elev = 0

plt.show(block=False)

i=31000
j=0
while (i < 35000): #min(len(odoPos.getAxis(0)), len(pureodoPos.getAxis(0)))):

    #ax.cla()
    xposition = odoPos.getAxis(0)[0:i]
    yposition = odoPos.getAxis(1)[0:i]
    zposition = odoPos.getAxis(2)[0:i]
    ax.plot(xposition, yposition, zposition, marker='*', label="Jacobian based Odometry + Reaction Forces", color=[0.0,0.8,0], alpha=0.5, lw=2)

    xposition = pureodoPos.getAxis(0)[0:i]
    yposition = pureodoPos.getAxis(1)[0:i]
    zposition = pureodoPos.getAxis(2)[0:i]
    ax.plot(xposition, yposition, zposition, marker='*', label="Jacobian based Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)
    plt.savefig('movie_stairs/anim_'+str(j)+'.png', format='png')
    print str(i)
    print str(j)
    i = i+50
    j = j+1



