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
odoPos.readData('/home/jhidalgocarrio/esa-npi/development/post-process_data/20131125-1505_asguard_sandfield/20131206-2344/data/odometry_position.0.data', cov=True)
odoPos.eigenValues()

# Odometry Pose
pureodoPos = data.ThreeData()
pureodoPos.readData('/home/jhidalgocarrio/esa-npi/development/post-process_data/20131125-1505_asguard_sandfield/20131207-0210/data/odometry_position.0.data', cov=True)
pureodoPos.eigenValues()

# Skid Odometry Pose
skidodoPos = data.ThreeData()
skidodoPos.readData('/home/jhidalgocarrio/esa-npi/development/post-process_data/20131125-1505_asguard_sandfield/20131206-2344/data/skid_odometry_position.0.data', cov=True)
skidodoPos.eigenValues()

# GPS/Vicon reference from Ground Truth
refPos = data.ThreeData()
refPos.readData('../post-process_data/20131125-1505_asguard_sandfield/20131206-2344/data/reference_position.0.data', cov=False)
refPos.eigenValues()

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
plt.plot(xposition, yposition, marker='.', linestyle='-', label="Planar Odometry", color=[0,0.5,1], lw=2)

rot = quat.quaternion([0.819, -0.014, 0.01001, -0.5735])
M = rot.toMatrix()
xposition = []
yposition = []

for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])

plt.plot(xposition, yposition, marker='D', linestyle='--', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-10, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':20})
plt.show(block=False)

savefig('figures/sandfield_position_x_y.png')


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

rot = quat.quaternion([0.819, -0.014, 0.01001, -0.5735])
M = rot.toMatrix()
xposition = []
zposition = []

for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    zposition.append(vec[2])

plt.plot(xposition, zposition, marker='D', linestyle='--', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)
plt.scatter(xposition[0], zposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], zposition[len(zposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], zposition[0]), xycoords='data',
                                xytext=(-70, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[len(xposition)-1], zposition[len(zposition)-1]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Elevation in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':20})
plt.show(block=False)

savefig('figures/sandfield_position_x_z.png')

#3D Plotting GPS values
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.set_title('Sandfield - Asguard')

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

rot = quat.quaternion([0.819, -0.014, 0.01001, -0.5735])
M = rot.toMatrix()
xposition = []
yposition = []
zposition = []

for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])
    zposition.append(vec[2])

ax.plot(xposition, yposition, zposition, marker='D', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)

ax.grid(True)
ax.legend(loc=1, prop={'size':20})
ax.set_xlabel(r'Position [$m$]')
ax.set_ylabel(r'Position [$m$]')
ax.set_zlabel(r'Elevation [$m$]')
ax.plot([-5.0, 20.0], [-5.0, 40.0], [-10, 10], linestyle='none') # One way to set axis limits instead set_xlimit
plt.show(block=False)

for i in range(0, 360):
    ax.azim = i
#   ax1.elev = 30 
    plt.savefig('movie_sandfield/anim_'+str(i)+'.png')

