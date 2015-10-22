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

# Skid Odometry Pose
skidodoPos = data.ThreeData()
skidodoPos.readData('../data/20130415_motion_model_test_track/20131206-2159/data/skid_odometry_position.0.data', cov=True)
skidodoPos.eigenValues()

#Odometry , Skid Odometry and GPS values(X-Y Axis Sand Field)
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1, figsize=(8,6))
ax = fig.add_subplot(111)

rot = quat.quaternion([0.99, 0.00, 0.0, 0.0261])
M = rot.toMatrix()
xposition = []
yposition = []

for i in range(0,len(odoPos.data)):
    x = odoPos.data[i][0]
    y = odoPos.data[i][1]
    z = odoPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])

xposition = xposition[0::50]
yposition = yposition[0::50]

ax.plot(xposition, yposition, marker='o', linestyle='-.', label= "Weighted Jacobian Odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)

xposition = []
yposition = []

for i in range(0,len(pureodoPos.data)):
    x = pureodoPos.data[i][0]
    y = pureodoPos.data[i][1]
    z = pureodoPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])

xposition = xposition[0::50]
yposition = yposition[0::50]

ax.plot(xposition, yposition, marker='x', linestyle='--', label="Jacobian Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

xposition = []
yposition = []

for i in range(0,len(skidodoPos.data)):
    x = skidodoPos.data[i][0]
    y = skidodoPos.data[i][1]
    z = skidodoPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])

xposition = xposition[0::50]
yposition = yposition[0::50]

ax.plot(xposition, yposition, marker='^', linestyle='-', label="Planar Odometry", color=[0,0.5,1], lw=2)

ax.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
ax.scatter(xposition[0], yposition[0], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
ax.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-30, -40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))
ax.annotate(r'End', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(+20, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
#plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

savefig('figures/testtrack_position_x_y.png')


#Odometry , Skid Odometry and GPS values(X-Z Axis Sand Field)
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(2)
ax = fig.add_subplot(111)

xposition = odoPos.getAxis(0)
zposition = odoPos.getAxis(2)
ax.plot(xposition, zposition, marker='o', linestyle='-.', label= "Weighted Jacobian Odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = pureodoPos.getAxis(0)
zposition = pureodoPos.getAxis(2)
ax.plot(xposition, zposition, marker='x', linestyle='--', label="Jacobian Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)
zposition = skidodoPos.getAxis(2)
ax.plot(xposition, zposition, marker='^', linestyle='-', label="Planar Odometry", color=[0,0.5,1], lw=2)

ax.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
ax.scatter(xposition[0], yposition[0], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
ax.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-20, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))
ax.annotate(r'End', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(+20, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
#plt.grid(True)
ax.legend(prop={'size':30})
plt.show(block=False)

savefig('figures/testtrack_position_x_z.png')

#3D Plotting values
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
ax = fig.add_subplot(111, projection='3d')
#ax.set_title('DFKI Test Track - Asguard')

#xposition = odoPos.getAxis(0)
#yposition = odoPos.getAxis(1)
#zposition = odoPos.getAxis(2)

xposition = []
yposition = []
zposition = []
for i in range(0,len(odoPos.data)):
    x = odoPos.data[i][0]
    y = odoPos.data[i][1]
    z = odoPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])
    zposition.append(vec[2])

xposition = xposition[0::50]
yposition = yposition[0::50]
zposition = zposition[0::50]

ax.plot(xposition, yposition, zposition, marker='o', linestyle='-.', label="Weighted Jacobian Odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)

#xposition = pureodoPos.getAxis(0)
#yposition = pureodoPos.getAxis(1)
#zposition = pureodoPos.getAxis(2)

xposition = []
yposition = []
zposition = []
for i in range(0,len(pureodoPos.data)):
    x = pureodoPos.data[i][0]
    y = pureodoPos.data[i][1]
    z = pureodoPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])
    zposition.append(vec[2])

xposition = xposition[0::50]
yposition = yposition[0::50]
zposition = zposition[0::50]

ax.plot(xposition, yposition, zposition, marker='x', linestyle='--', label="Jacobian Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

#xposition = skidodoPos.getAxis(0)
#yposition = skidodoPos.getAxis(1)
#zposition = skidodoPos.getAxis(2)
xposition = []
yposition = []
zposition = []
for i in range(0,len(skidodoPos.data)):
    x = skidodoPos.data[i][0]
    y = skidodoPos.data[i][1]
    z = skidodoPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])
    zposition.append(vec[2])

xposition = xposition[0::50]
yposition = yposition[0::50]
zposition = zposition[0::50]

ax.plot(xposition, yposition, zposition, marker='^', linestyle='-', label="Planar Odometry", color=[0,0.5,1], lw=2)


ax.scatter(xposition[0], yposition[0], zposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
xstart, ystart, _ = proj3d.proj_transform(xposition[0], yposition[0], zposition[0], ax.get_proj())
start_label = ax.annotate(r'Start', xy=(xstart, ystart), xycoords='data',
                    xytext=(-20, -40), textcoords='offset points', fontsize=22,
                    #bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
                    arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=.2', lw=2.0))

ax.scatter(xposition[0], yposition[0], zposition[0], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
xend, yend, _ = proj3d.proj_transform(xposition[0], yposition[0], zposition[0], ax.get_proj())
end_label = ax.annotate(r'End', xy=(xend, yend), xycoords='data',
                    xytext=(+20, +40), textcoords='offset points', fontsize=22,
                    #bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

ax.grid(True)
ax.legend(prop={'size':30})
ax.set_xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
ax.set_ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
ax.set_zlabel(r'Z [$m$]', fontsize=35, fontweight='bold')
ax.plot([-5.0, 60.0], [-8.0, 4.0], [-1.0, 5.0], linestyle='none') # One way to set axis limits instead set_xlimit

def update_position(e):
    x2, y2, _ = proj3d.proj_transform(xposition[0], yposition[0], zposition[0], ax.get_proj())
    start_label.xy = x2,y2
    start_label.update_positions(fig.canvas.renderer)
    end_label.xy = x2,y2
    end_label.update_positions(fig.canvas.renderer)
    fig.canvas.draw()

fig.canvas.mpl_connect('button_release_event', update_position)

plt.show(block=False)


savefig('figures/testtrack_position_3d.png')

for i in range(0, 360):
    ax.azim = i
#   ax1.elev = 30 
    plt.savefig('movie_testtrack/anim_'+str(i)+'.png')

