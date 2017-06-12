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

matplotlib.style.use('classic') #in matplotlib >= 1.5.1

# Odometry + Support polygon Pose
odoPos = data.ThreeData()
odoPos.readData('~/npi/data/20131125-1505_asguard_sandfield/20131206-2344/data/odometry_position.0.data', cov=True)
odoPos.eigenValues()

# Odometry Pose
pureodoPos = data.ThreeData()
pureodoPos.readData('~/npi/data/20131125-1505_asguard_sandfield/20131207-0210/data/odometry_position.0.data', cov=True)
pureodoPos.eigenValues()

# Skid Odometry Pose
skidodoPos = data.ThreeData()
skidodoPos.readData('~/npi/data/20131125-1505_asguard_sandfield/20131206-2344/data/skid_odometry_position.0.data', cov=True)
skidodoPos.eigenValues()

# GPS/Vicon reference from Ground Truth
refPos = data.ThreeData()
refPos.readData('~/npi/data/20131125-1505_asguard_sandfield/20131206-2344/data/reference_position.0.data', cov=False)
refPos.eigenValues()


#Odometry , Skid Odometry and GPS values(X-Y Axis Sand Field)
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering

xposition = odoPos.getAxis(0)[0::50]
yposition = odoPos.getAxis(1)[0::50]
ax.plot(xposition, yposition, marker='o', linestyle='-.', label= "3D odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = pureodoPos.getAxis(0)[0::50]
yposition = pureodoPos.getAxis(1)[0::50]
ax.plot(xposition, yposition, marker='x', linestyle='--', label="contact point odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)[0::50]
yposition = skidodoPos.getAxis(1)[0::50]
ax.plot(xposition, yposition, marker='^', linestyle='-', label="skid odometry", color=[0,0.5,1], lw=2)

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

xposition = xposition[0::50]
yposition = yposition[0::50]

ax.plot(xposition, yposition, marker='D', linestyle='--', label="reference trajectory", color=[0.5,0,0], alpha=0.5, lw=2)
ax.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
ax.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
ax.annotate('Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-10, -40), textcoords='offset points', fontsize=30,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))
ax.annotate('End', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=30,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(prop={'size':30})
plt.show(block=False)

savefig('sandfield_position_x_y.pdf')


#Odometry , Skid Odometry and GPS values(X-Z Axis Sand Field)
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(2, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering

xposition = odoPos.getAxis(0)[0::50]
zposition = odoPos.getAxis(2)[0::50]
ax.plot(xposition, zposition, marker='o', linestyle='-.', label= "3D odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = pureodoPos.getAxis(0)[0::50]
zposition = pureodoPos.getAxis(2)[0::50]
ax.plot(xposition, zposition, marker='x', linestyle='--', label="contact point odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)[0::50]
zposition = skidodoPos.getAxis(2)[0::50]
ax.plot(xposition, zposition, marker='^', linestyle='-', label="skid odometry", color=[0,0.5,1], lw=2)

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

xposition = xposition[0::50]
zposition = zposition[0::50]

ax.plot(xposition, zposition, marker='D', linestyle='--', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)
ax.scatter(xposition[0], zposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
ax.scatter(xposition[len(xposition)-1], zposition[len(zposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
ax.annotate(r'Start', xy=(xposition[0], zposition[0]), xycoords='data',
                                xytext=(-70, -30), textcoords='offset points', fontsize=30,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
ax.annotate(r'End', xy=(xposition[len(xposition)-1], zposition[len(zposition)-1]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=30,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(prop={'size':20})
plt.show(block=False)

savefig('sandfield_position_x_z.png')

#3D Plotting GPS values
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})

ax = fig.add_subplot(111, projection='3d')
#ax.set_title('Sandfield - Asguard')

xposition = odoPos.getAxis(0)[0::50]
yposition = odoPos.getAxis(1)[0::50]
zposition = odoPos.getAxis(2)[0::50]
ax.plot(xposition, yposition, zposition, marker='o', linestyle='-.', label="3D odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)

xposition = pureodoPos.getAxis(0)[0::50]
yposition = pureodoPos.getAxis(1)[0::50]
zposition = pureodoPos.getAxis(2)[0::50]
ax.plot(xposition, yposition, zposition, marker='x', linestyle='--', label="contact point odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

xposition = skidodoPos.getAxis(0)[0::50]
yposition = skidodoPos.getAxis(1)[0::50]
zposition = skidodoPos.getAxis(2)[0::50]
ax.plot(xposition, yposition, zposition, marker='^', linestyle='-', label="skid odometry", color=[0,0.5,1], lw=2)

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

xposition = xposition[0::50]
yposition = yposition[0::50]
zposition = zposition[0::50]


ax.plot(xposition, yposition, zposition, marker='D', linestyle='--',
        label="reference trajectory", color=[0.5,0,0], alpha=0.5, lw=2)

ax.scatter(xposition[0], yposition[0], zposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
xstart, ystart, _ = proj3d.proj_transform(xposition[0], yposition[0], zposition[0], ax.get_proj())
start_label = ax.annotate(r'Start', xy=(xstart, ystart), xycoords='data',
                    xytext=(-50, -40), textcoords='offset points', fontsize=30,
                    #bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
                    arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=.2', lw=2.0))

ax.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], zposition[len(zposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
xend, yend, _ = proj3d.proj_transform(xposition[len(xposition)-1], yposition[len(yposition)-1], zposition[len(zposition)-1], ax.get_proj())
end_label = ax.annotate(r'End', xy=(xend, yend), xycoords='data',
                    xytext=(+20, +40), textcoords='offset points', fontsize=30,
                    #bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))


ax.grid(True)
#ax.legend(loc=2, prop={'size':30})
ax.set_xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
ax.set_ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
ax.set_zlabel(r'Z [$m$]', fontsize=35, fontweight='bold')
ax.plot([-5.0, 20.0], [-5.0, 40.0], [-5, 5], linestyle='none') # One way to set axis limits instead set_xlimit

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

savefig('sandfield_position_3d.pdf')

for i in range(0, 360):
    ax.azim = i
#   ax1.elev = 30 
    plt.savefig('movie_sandfield/anim_'+str(i)+'.png')

