#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2017-06-12 12:57:38
#####################################
# Percentage final error for asguard
#####################################
import numpy as np
from pylab import *
import matplotlib.pyplot as plt

matplotlib.style.use('classic') #in matplotlib >= 1.5.1

N = 3
ind = np.arange(N)  # the x locations for the groups
width = 0.27       # the width of the bars

matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111) # Create matplotlib axes

#motocross
yvals = [8.33, 3.48, 2.78]
rects1 = ax.bar(ind, yvals, width, color='black')

#DFKI track
zvals = [5.06, 4.74, 4.17]
rects2 = ax.bar(ind+width, zvals, width, color='grey')

#Sandfield
kvals = [6.15, 4.21, 2.11]
rects3 = ax.bar(ind+width*2, kvals, width, color='lightgray')

ax.set_ylabel(r'distance error [$\%$]', fontsize=40, fontweight='bold', color="black")
ax.set_xticks(ind+width)
ax.set_xticklabels( ('skid odometry', 'contact point odometry', 'enhanced 3D odometry'), 
        fontsize=35, fontweight='bold', color="black")
ax.legend( (rects3[0], rects1[0], rects2[0]), ('sandfield',  'motocross', 'DFKI track'),
        prop={'size':30})
ax.grid(True)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

def autolabel(rects):
    for rect in rects:
        h = rect.get_height()
        ax.text(rect.get_x()+rect.get_width()/2., 1.0*h, '%.2f'%float(h),
                ha='center', va='bottom', fontsize=30)

autolabel(rects1)
autolabel(rects2)
autolabel(rects3)

savefig('barchart_percentage_odometry_methods_asguard.pdf')
plt.show(block=True)

#####################################
# RMSE final error for asguard
#####################################
import numpy as np
from pylab import *
import matplotlib.pyplot as plt

matplotlib.style.use('classic') #in matplotlib >= 1.5.1

N = 3
ind = np.arange(N)  # the x locations for the groups
width = 0.27       # the width of the bars

matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig = plt.figure(2, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111) # Create matplotlib axes

#motocross
yvals = [13.96, 7.09, 7.01]
rects_moto = ax.bar(ind, yvals, width, color='black')

#Sandfield
zvals = [3.64, 2.81, 1.42]
rects_sand = ax.bar(ind+width, zvals, width, color='grey')

ax.set_ylim(-0.0, 15)
ax.set_ylabel(r'RMSE [$m$]', fontsize=40, fontweight='bold', color="black")
ax.set_xticks(ind+width)
ax.set_xticklabels( ('skid odometry', 'contact point odometry', 'enhanced 3D odometry'), 
        fontsize=35, fontweight='bold', color="black")
ax.legend( (rects_sand[0], rects_moto[0]), ('sandfield',  'motocross'),
        prop={'size':30})
ax.grid(True)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

def autolabel(rects):
    for rect in rects:
        h = rect.get_height()
        ax.text(rect.get_x()+rect.get_width()/2., 1.0*h, '%.2f'%float(h),
                ha='center', va='bottom', fontsize=30)

autolabel(rects_moto)
autolabel(rects_sand)

savefig('barchart_rmse_odometry_methods_asguard.pdf')
plt.show(block=True)

