#!/usr/bin/env python

path = '/home/javi/flatfish/development/data/20150312_dagon_estimator_evaluation_1h/20150312-1715/plots/'

#######################################
path_odometry_file = path + 'trajectory_odo.csv'

path_reference_file = path + 'trajectory_gt.csv'
#######################################

import sys
sys.path.insert(0, './src/core')
import csv
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov



#Dagon Odometry
odometry = data.ThreeData()
odometry.readData(path_odometry_file, cov=False)
odometry.eigenValues()

#Reference Pose
reference = data.ThreeData()
reference.readData(path_reference_file, cov=False)
reference.eigenValues()

################################
### COMPUTE COV EIGENVALUES  ###
################################
odometry.covSymmetry()
odometry.eigenValues()
reference.covSymmetry()
reference.eigenValues()

############
### PLOT ###
############
:matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

# Display Odometry trajectory
position = np.column_stack((odometry.getAxis(0)[0::50], odometry.getAxis(1)[0::50],  odometry.getAxis(2)[0::50]))

# Display Odometry trajectory
x = position[:,0]
y = position[:,1]

Q = ax.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy',
        angles='xy', scale=1, color='b', units='x', linewidths=(1,),
        edgecolors=('k'), headaxislength=5)
qk = ax.quiverkey(Q, 0.9, 0.06, 0.5,  'dead-reckoning', fontproperties={'weight': 'bold', 'size':16})


import os
from matplotlib.cbook import get_sample_data
from matplotlib._png import read_png
import matplotlib.image as image
from scipy import ndimage
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
fn = get_sample_data(os.getcwd()+"/data/img/dagon.png", asfileobj=False)
dagon = image.imread(fn)
dagon = ndimage.rotate(dagon, 180)
imdagon = OffsetImage(dagon, zoom=0.5)


abstart = AnnotationBbox(imdagon, xy=(x[0], y[0]),
                        xybox=None,
                        xycoords='data',
                        boxcoords="offset points",
                        frameon=False)

dagon = ndimage.rotate(dagon, -180)
imdagon = OffsetImage(dagon, zoom=0.5)


abend = AnnotationBbox(imdagon, xy=(x[len(x)-1], y[len(x)-1]),
                        xybox=None,
                        xycoords='data',
                        boxcoords="offset points",
                        frameon=False)

ax.annotate("Start", xy=(x[0], y[0]), xycoords='data',
                                xytext=(-60, 65), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

ax.annotate("End", xy=(x[len(x)-1], y[len(x)-1]), xycoords='data',
                                xytext=(-160, 0), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

ax.add_artist(abstart)
ax.add_artist(abend)

# Display Ground Truth trajectory
position = np.column_stack((reference.getAxis(0)[0::50], reference.getAxis(1)[0::50],  reference.getAxis(2)[0::50]))

# Display Ground Truth trajectory
x = position[:,0]
y = position[:,1]

Q = ax.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy',
        angles='xy', scale=1, color='r', units='x', linewidths=(1,),
        edgecolors=('k'), headaxislength=5)
qk = ax.quiverkey(Q, 0.9, 0.02, 0.5,  'ground truth', fontproperties={'weight': 'bold', 'size':16})


plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(handles=[dagon], loc=1, prop={'size':30})
plt.show(block=False)


