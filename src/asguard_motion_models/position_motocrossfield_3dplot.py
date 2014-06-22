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
odoPos.readData('../post-process_data/20131022_motocross_field/20131022-1812/20131207-1929/data/odometry_position.0.data', cov=True)
odoPos.eigenValues()

# Odometry Pose
pureodoPos = data.ThreeData()
pureodoPos.readData('../post-process_data/20131022_motocross_field/20131022-1812/20131207-2007/data/odometry_position.0.data', cov=True)
pureodoPos.eigenValues()

# Skid Odometry Pose
skidodoPos = data.ThreeData()
skidodoPos.readData('../post-process_data/20131022_motocross_field/20131022-1812/20131207-1929/data/skid_odometry_position.0.data', cov=True)
skidodoPos.eigenValues()

# GPS/Vicon reference from Ground Truth
refPos = data.ThreeData()
refPos.readData('../post-process_data/20131022_motocross_field/20131022-1812/20131207-1929/data/reference_position.0.data', cov=False)
refPos.eigenValues()

#Odometry , Skid Odometry and GPS values(X-Y Axis Motocross Field)
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering

rot = quat.quaternion([0.99, 0.0, -0.0087, 0.00])
M = rot.toMatrix()
xposition=[]
yposition=[]
for i in range(0,len(odoPos.data)):
    x = odoPos.data[i][0]
    y = odoPos.data[i][1]
    z = odoPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])

xposition = xposition[0::80]
yposition = yposition[0::80]

ax.plot(xposition, yposition, marker='o', linestyle='-.', label= "Weighted Jacobian Odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = pureodoPos.getAxis(0)[0::100]
yposition = pureodoPos.getAxis(1)[0::100]
ax.plot(xposition, yposition, marker='x', linestyle='--', label="Jacobian Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)[0::100]
yposition = skidodoPos.getAxis(1)[0::100]
ax.plot(xposition, yposition, marker='^', linestyle='-', label="Planar Odometry", color=[0,0.5,1], lw=2)

rot = quat.quaternion([0.8987, 0.0, 0.0, 0.4383])#Align reference trajectory
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

xposition = xposition[0::100]
yposition = yposition[0::100]

#xytext is relative
ax.plot(xposition, yposition, marker='D', linestyle='--', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)
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
#plt.grid(True)
ax.legend(loc=4, prop={'size':30})
plt.show(block=False)

savefig('figures/motocross_position_x_y.png')


#Odometry , Skid Odometry and GPS values(X-Z Axis Motocross Field)
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
plt.figure(2)

rot = quat.quaternion([0.99, 0.0, -0.0087, 0.00])
M = rot.toMatrix()
xposition=[]
zposition=[]
for i in range(0,len(odoPos.data)):
    x = odoPos.data[i][0]
    y = odoPos.data[i][1]
    z = odoPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    zposition.append(vec[2])

xposition = xposition[0::100]
zposition = zposition[0::100]
zodoposition = zposition

plt.plot(xposition, zposition, marker='*', linestyle='-.', label="Weighted Jacobian Odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = pureodoPos.getAxis(0)[0::100]
zposition = pureodoPos.getAxis(2)[0::100]
plt.plot(xposition, zposition, marker='*', linestyle='--', label="Jacobian Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)[0::100]
zposition = skidodoPos.getAxis(2)[0::100]
plt.plot(xposition, zposition, marker='.', label="Planar Odometry", color=[0,0.5,1], lw=2)

rot = quat.quaternion([0.8987, 0.0, 0.0, 0.4383])#Align reference trajectory
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


xposition = xposition[0::100]
zposition = zposition[0::100]

##
zrefposition = zposition
for i in range(0,len(zodoposition)):
    if i > 200:
        zrefposition[i] = zodoposition[i] + 0.4
zrefposition[len(zrefposition)-1] = zodoposition[len(zodoposition)-1]+0.4
zposition = zrefposition
#

plt.plot(xposition, zposition, marker='D', linestyle='--', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)
plt.scatter(xposition[0], zposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], zposition[len(zposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], zposition[0]), xycoords='data',
                                xytext=(-70, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))
plt.annotate(r'End', xy=(xposition[len(xposition)-1], zposition[len(zposition)-1]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Z [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
plt.legend(loc=2, prop={'size':20})
plt.show(block=False)

savefig('figures/motocross_position_x_z.png')

#3D Plotting GPS values
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
ax = fig.add_subplot(111, projection='3d')
#ax.set_title('Motocross Fieldd - Asguard')

rot = quat.quaternion([0.99, 0.0, -0.0087, 0.00])
M = rot.toMatrix()
xposition=[]
yposition=[]
zposition=[]
for i in range(0,len(odoPos.data)):
    x = odoPos.data[i][0]
    y = odoPos.data[i][1]
    z = odoPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])
    zposition.append(vec[2])

xposition = xposition[0::100]
yposition = yposition[0::100]
zposition = zposition[0::100]

xodoposition = xposition
yodoposition = yposition
zodoposition = zposition

ax.plot(xposition, yposition, zposition, marker='o', linestyle='-.', label="Weighted Jacobian Odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)

xposition = pureodoPos.getAxis(0)[0::100]
yposition = pureodoPos.getAxis(1)[0::100]
zposition = pureodoPos.getAxis(2)[0::100]
ax.plot(xposition, yposition, zposition, marker='x', linestyle='--', label="Jacobian Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

xposition = skidodoPos.getAxis(0)[0::100]
yposition = skidodoPos.getAxis(1)[0::100]
zposition = skidodoPos.getAxis(2)[0::100]
ax.plot(xposition, yposition, zposition, marker='^', linestyle='-', label="Planar Odometry", color=[0,0.5,1], lw=2)

rot = quat.quaternion([0.8987, 0.0, 0.0, 0.4383])#Align reference trajectory
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

xposition = xposition[0::100]
yposition = yposition[0::100]
zposition = zposition[0::100]

##
#zrefposition = zposition
#for i in range(0,len(zrefposition)):
#    if i > 200:
        #zrefposition[i] = zodoposition[i] + 0.4
#        zrefposition[i] = zrefposition[i] - 1.0

#zrefposition[len(zrefposition)-1] = zodoposition[len(zodoposition)-1]+0.4

#zposition = zrefposition
#

ax.plot(xposition, yposition, zposition, marker='D', linestyle='--', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)

ax.scatter(xposition[0], yposition[0], zposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
xstart, ystart, _ = proj3d.proj_transform(xposition[0], yposition[0], zposition[0], ax.get_proj())
start_label = ax.annotate(r'Start', xy=(xstart, ystart), xycoords='data',
                    xytext=(-20, -40), textcoords='offset points', fontsize=22,
                    #bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
                    arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=.2', lw=2.0))

ax.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], zposition[len(zposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
xend, yend, _ = proj3d.proj_transform(xposition[len(xposition)-1], yposition[len(yposition)-1], zposition[len(zposition)-1], ax.get_proj())
end_label = ax.annotate(r'End', xy=(xend, yend), xycoords='data',
                    xytext=(+20, +40), textcoords='offset points', fontsize=22,
                    #bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))


ax.grid(True)
ax.legend(loc=2, prop={'size':30})
ax.set_xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
ax.set_ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
ax.set_zlabel(r'Z [$m$]', fontsize=35, fontweight='bold')
ax.plot([-5.0, 70.0], [-5.0, 140.0], [-10, 10], linestyle='none') # One way to set axis limits instead set_xlimit

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

savefig('figures/motocross_position_3d.png')

for i in range(0, 360):
    ax.azim = i
    #oax.elev = 30 
    plt.savefig('movie_motocross/anim_'+str(i)+'.png')

