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
odoPos.readData('../post-process_data/20130415_motion_model_test_track/20131206-2159/data/odometry_position.0.data', cov=True)
odoPos.eigenValues()

# Odometry Pose
pureodoPos = data.ThreeData()
pureodoPos.readData('../post-process_data/20130415_motion_model_test_track/20131206-2243/data/odometry_position.0.data', cov=True)
pureodoPos.eigenValues()

# Skid Odometry Pose
skidodoPos = data.ThreeData()
skidodoPos.readData('../post-process_data/20130415_motion_model_test_track/20131206-2159/data/skid_odometry_position.0.data', cov=True)
skidodoPos.eigenValues()

#Odometry , Skid Odometry and GPS values(X-Y Axis Sand Field)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(1)

xposition = odoPos.getAxis(0)
yposition = odoPos.getAxis(1)
plt.plot(xposition, yposition, marker='*', linestyle='-.', label="Jacobian based Odometry + Reaction Forces", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = pureodoPos.getAxis(0)
yposition = pureodoPos.getAxis(1)
plt.plot(xposition, yposition, marker='*', linestyle='--', label="Jacobian based Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)
yposition = skidodoPos.getAxis(1)
plt.plot(xposition, yposition, marker='.', label="Planar Odometry", color=[0,0.5,1], lw=2)

plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-20, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(+20, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(loc=1, prop={'size':20})
plt.show(block=False)

savefig('figures/testtrack_position_x_y.png')


#Odometry , Skid Odometry and GPS values(X-Z Axis Sand Field)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(2)

xposition = odoPos.getAxis(0)
zposition = odoPos.getAxis(2)
plt.plot(xposition, zposition, marker='*', linestyle='-.', label="Jacobian based Odometry + Reaction Forces", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = pureodoPos.getAxis(0)
zposition = pureodoPos.getAxis(2)
plt.plot(xposition, zposition, marker='*', linestyle='--', label="Jacobian based Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)
zposition = skidodoPos.getAxis(2)
plt.plot(xposition, zposition, marker='.', label="Planar Odometry", color=[0,0.5,1], lw=2)

plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-20, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(+20, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))


plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Elevation in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(loc=1, prop={'size':20})
plt.show(block=False)

savefig('figures/testtrack_position_x_z.png')

#3D Plotting values
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.set_title('DFKI Test Track - Asguard')

xposition = odoPos.getAxis(0)
yposition = odoPos.getAxis(1)
zposition = odoPos.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='*', label="Jacobian based Odometry + Reaction Forces", color=[0.0,0.8,0], alpha=0.5, lw=2)

xposition = pureodoPos.getAxis(0)
yposition = pureodoPos.getAxis(1)
zposition = pureodoPos.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='*', label="Jacobian based Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

xposition = skidodoPos.getAxis(0)
yposition = skidodoPos.getAxis(1)
zposition = skidodoPos.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='.', label="Planar Odometry", color=[0,0.5,1], lw=2)

ax.grid(True)
ax.legend(prop={'size':20})
ax.set_xlabel(r'Position [$m$]')
ax.set_ylabel(r'Position [$m$]')
ax.set_zlabel(r'Elevation [$m$]')
ax.plot([-5.0, 60.0], [-10.0, 4.0], [-1.0, 5.0], linestyle='none') # One way to set axis limits instead set_xlimit
plt.show(block=False)

for i in range(0, 360):
    ax.azim = i
#   ax1.elev = 30 
    plt.savefig('movie_testtrack/anim_'+str(i)+'.png')

