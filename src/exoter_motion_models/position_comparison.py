#!/usr/bin/env python

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
odometry.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140600_pink_odometry_test/20140630-1847/pose_odo_position.0.data', cov=True)
odometry.eigenValues()

#Vicon Pose
vicon = data.ThreeData()
vicon.readData('/home/jhidalgocarrio/exoter/development/post-process_data/20140600_pink_odometry_test/20140630-1847/pose_ref_position.0.data', cov=False)
vicon.eigenValues()

#Position comparison
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
xposition = odometry.getAxis(0)
yposition = odometry.getAxis(1)
ax.plot(xposition, yposition, marker='^', linestyle='-', label="Odometry Pose", color=[0,0.5,1], lw=2)

xposition = odometry.getAxis(0)[0::10]
yposition = odometry.getAxis(1)[0::10]
xycov = odometry.getCov(1)[0::10]
for i in range(0, len(xycov)):
    cov.plot_cov_ellipse(xycov[i], pos=[xposition[i], yposition[i]],
                    linewidth=2, alpha=0.5, facecolor='green', edgecolor='black')

xposition = vicon.getAxis(0)
yposition = vicon.getAxis(1)
ax.plot(xposition, yposition, marker='D', linestyle='--', label="Reference Pose", color=[0.5,0,0], alpha=0.5, lw=2)
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
plt.grid(True)
ax.legend(loc=4, prop={'size':30})
plt.show(block=False)




