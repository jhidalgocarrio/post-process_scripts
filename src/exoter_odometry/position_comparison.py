#!/usr/bin/env python

#######################################
#path_odometry_file = '/home/jhidalgocarrio/exoter/development/data/20140600_pink_odometry/20140630-1847/pose_odo_position.0.data'
#path_odometry_file = '/home/jhidalgocarrio/exoter/development/data/20140723_pink_odometry/20140723-1845/pose_odo_position.0.data'
#path_odometry_file = '/home/jhidalgocarrio/exoter/development/data/20141023_pink_test/20141023-2011/pose_odo_position.0.data'
path_odometry_file = '/home/jhidalgocarrio/exoter/development/data/20140911_decos_field/20140911-1805/pose_odo_position.0.data'

#path_reference_file = '/home/jhidalgocarrio/exoter/development/data/20140600_pink_odometry/20140630-1847/pose_ref_position.0.data'
#path_reference_file = '/home/jhidalgocarrio/exoter/development/data/20140723_pink_odometry/20140723-1845/pose_ref_position.0.data'
#path_reference_file = '/home/jhidalgocarrio/exoter/development/data/20141023_pink_test/20141023-2011/pose_ref_position.0.data'
path_reference_file = '/home/jhidalgocarrio/exoter/development/data/20140911_decos_field/20140911-1805/pose_ref_position.0.data'
#######################################


import sys
sys.path.insert(0, './src/core')
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov


#ExoTeR Odometry
odometry = data.ThreeData()
odometry.readData(path_odometry_file, cov=True)
odometry.eigenValues()

#Vicon Pose
vicon = data.ThreeData()
vicon.readData(path_reference_file, cov=True)
vicon.eigenValues()

#Position comparison X-Y plane
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
xposition = odometry.getAxis(0)
yposition = odometry.getAxis(1)
ax.plot(xposition, yposition, marker='^', linestyle='-', label="Odometry pose", color=[0.34,0,1], lw=2)

xposition = odometry.getAxis(0)[0::10]
yposition = odometry.getAxis(1)[0::10]
xycov = odometry.getCov(1)[0::10]
for i in range(0, len(xycov)):
    cov.plot_cov_ellipse(xycov[i], pos=[xposition[i], yposition[i]], nstd=3,
                    linewidth=2, alpha=0.5, facecolor='green', edgecolor='black')

xposition = vicon.getAxis(0)
yposition = vicon.getAxis(1)
ax.plot(xposition, yposition, marker='D', linestyle='--', label="Reference pose", color=[0.5,0,0], alpha=0.5, lw=2)
ax.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
ax.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
ax.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-40, -40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))
ax.annotate(r'End', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(-40, +40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

xposition = vicon.getAxis(0)[0::100]
yposition = vicon.getAxis(1)[0::100]
xycov = vicon.getCov(1)[0::100]
for i in range(0, len(xycov)):
    cov.plot_cov_ellipse(xycov[i], pos=[xposition[i], yposition[i]], nstd=3,
                    linewidth=2, alpha=0.2, facecolor=[0.4,0,0.4], edgecolor='black')


plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=4, prop={'size':30})
plt.show(block=False)

savefig('odometry_exoter_different_frequencies_x_y_zoom2.png')

#3D Plotting values
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
ax = fig.add_subplot(111, projection='3d')


xposition = odometry.getAxis(0)
yposition = odometry.getAxis(1)
zposition = odometry.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='o', linestyle='-.', label="Weighted Jacobian Odometry", color=[0.0,0.8,0], alpha=0.5, lw=2)

xposition = vicon.getAxis(0)
yposition = vicon.getAxis(1)
zposition = vicon.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='D', linestyle='--', label="Reference pose", color=[0.5,0,0], alpha=0.5, lw=2)

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


