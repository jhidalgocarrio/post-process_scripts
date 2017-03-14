import sys
sys.path.insert(0, './src/core')
import datadisplay as data
import csv, scipy
import math
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

#matplotlib.style.use('ggplot') #in matplotlib >= 1.5.1

def adjust_spines(ax,spines):
    for loc, spine in ax.spines.items():
        if loc in spines:
            spine.set_position(('outward',10)) # outward by 10 points
            spine.set_smart_bounds(True)
        else:
            spine.set_color('none') # don't draw spine

    # turn off ticks where there is no spine
    if 'left' in spines:
        ax.yaxis.set_ticks_position('left')
    else:
        # no yaxis ticks
        ax.yaxis.set_ticks([])

    if 'bottom' in spines:
        ax.xaxis.set_ticks_position('bottom')
    else:
        # no xaxis ticks
        ax.xaxis.set_ticks([])

def trunc(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    return ('%.*f' % (n + 1, f))[:-1]


# Data values computed at 2012
speed_mission_avg=[1.2048192771, 5.8708414873, 18.4645286686, 19.1780821918, 120]
speed_loco_max_avg = [1.20, 92, 90, 90, 120]
speed_loco_nominal_avg = [1.20, 10, 30, 50, 120]

# Plot the values
matplotlib.rcParams.update({'font.size': 25})
fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_title('Average Speed per Sol', fontsize=25, fontweight='bold')
ax.set_xlim(-1.0, 50)
ax.set_ylim(-1.0, 130)
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.spines['bottom'].set_position(('data',0))
ax.yaxis.set_ticks_position('left')
ax.spines['left'].set_position(('data',0))
ax.grid(True)
adjust_spines(ax,['left', 'bottom'])

#Mission average velocities
xposition = [1, 10, 20, 30 , 40]
yposition = speed_mission_avg
plt.plot(xposition, yposition, marker='.', linestyle='-', color="blue", lw=2)
plt.plot(xposition[0], yposition[0], linestyle='none', marker='D', color='blue', markersize=10)
plt.plot(xposition[1], yposition[1], linestyle='none', marker='D', color='red', markersize=10)
plt.plot(xposition[2], yposition[2], linestyle='none', marker='D', color='yellow', markersize=10)
plt.plot(xposition[3], yposition[3], linestyle='none', marker='D', color='green', markersize=10)
plt.plot(xposition[4], yposition[4], linestyle='none', marker='D', color='magenta', markersize=10)


plt.annotate(r''+trunc(yposition[0],1), xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]), xycoords='data',
                                xytext=(10, -15), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3], yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[4],1), xy=(xposition[4], yposition[4]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)

#Maximum average velocity
yposition = speed_loco_max_avg
plt.plot(xposition, yposition, marker='.', linestyle='--', color="red", lw=2)
plt.plot(xposition[0], yposition[0], linestyle='none', marker='D', color='blue', markersize=10)
plt.plot(xposition[1], yposition[1], linestyle='none', marker='D', color='red', markersize=10)
plt.plot(xposition[2], yposition[2], linestyle='none', marker='D', color='yellow', markersize=10)
plt.plot(xposition[3], yposition[3], linestyle='none', marker='D', color='green', markersize=10)
plt.plot(xposition[4], yposition[4], linestyle='none', marker='D', color='magenta', markersize=10)

plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3], yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points', fontsize=22)

yposition = speed_loco_nominal_avg
plt.plot(xposition, yposition, marker='.', linestyle='-.', color="green", lw=2)
plt.plot(xposition[0], yposition[0], linestyle='none', marker='D', color='blue', markersize=10)
plt.plot(xposition[1], yposition[1], linestyle='none', marker='D', color='red', markersize=10)
plt.plot(xposition[2], yposition[2], linestyle='none', marker='D', color='yellow', markersize=10)
plt.plot(xposition[3], yposition[3], linestyle='none', marker='D', color='green', markersize=10)
plt.plot(xposition[4], yposition[4], linestyle='none', marker='D', color='magenta', markersize=10)


plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3], yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points', fontsize=22)


plt.ylabel(r' Speed [$m/sol$]', fontsize=25, fontweight='bold')
plt.legend(prop={'size':25})
setp( ax.get_xticklabels(), visible=False)
#savefig('data/speed_missions_comparison_2012.png')
plt.show(block=True)

# Data values computed at 2013
speed_mission_avg=[1.2048192771, 5.8708414873, 9.2, 19.1780821918, 210]
speed_loco_max_avg = [1.20, 92, 100.0, 100.0, 210]
speed_loco_nominal_avg = [1.20, 10, 43, 50, 210]

# Plot the values
matplotlib.rcParams.update({'font.size': 25, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)
#ax.set_title('Average Speed per Sol', fontsize=25, fontweight='bold')
ax.set_xlim(-1.0, 50)
ax.set_ylim(-1.0, 220)
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.spines['bottom'].set_position(('data',0))
ax.yaxis.set_ticks_position('left')
ax.spines['left'].set_position(('data',0))
ax.grid(True)
adjust_spines(ax,['left', 'bottom'])

#Mission average velocities
xposition = [1, 10, 20, 30 , 40]
yposition = speed_mission_avg
plt.plot(xposition, yposition, marker='.', linestyle='-', color="blue", lw=2)
plt.plot(xposition[0], yposition[0], linestyle='none', marker='D', color='blue', markersize=10)
plt.plot(xposition[1], yposition[1], linestyle='none', marker='D', color='red', markersize=10)
plt.plot(xposition[2], yposition[2], linestyle='none', marker='D', color='yellow', markersize=10)
plt.plot(xposition[3], yposition[3], linestyle='none', marker='D', color='green', markersize=10)
plt.plot(xposition[4], yposition[4], linestyle='none', marker='D', color='magenta', markersize=10)


plt.annotate(r''+trunc(yposition[0],1), xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]), xycoords='data',
                                xytext=(10, -15), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3], yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[4],1), xy=(xposition[4], yposition[4]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)

#Maximum average velocity
yposition = speed_loco_max_avg
plt.plot(xposition, yposition, marker='.', linestyle='--', color="red", lw=2)
plt.plot(xposition[0], yposition[0], linestyle='none', marker='D', color='blue', markersize=10)
plt.plot(xposition[1], yposition[1], linestyle='none', marker='D', color='red', markersize=10)
plt.plot(xposition[2], yposition[2], linestyle='none', marker='D', color='yellow', markersize=10)
plt.plot(xposition[3], yposition[3], linestyle='none', marker='D', color='green', markersize=10)
plt.plot(xposition[4], yposition[4], linestyle='none', marker='D', color='magenta', markersize=10)

plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3], yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points', fontsize=22)

yposition = speed_loco_nominal_avg
plt.plot(xposition, yposition, marker='.', linestyle='-.', color="green", lw=2)
plt.plot(xposition[0], yposition[0], linestyle='none', marker='D', color='blue', markersize=10)
plt.plot(xposition[1], yposition[1], linestyle='none', marker='D', color='red', markersize=10)
plt.plot(xposition[2], yposition[2], linestyle='none', marker='D', color='yellow', markersize=10)
plt.plot(xposition[3], yposition[3], linestyle='none', marker='D', color='green', markersize=10)
plt.plot(xposition[4], yposition[4], linestyle='none', marker='D', color='magenta', markersize=10)


plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points', fontsize=22)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3], yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points', fontsize=22)


plt.ylabel(r' Speed [$m/sol$]', fontsize=35,  fontweight='bold')
plt.legend(prop={'size':35})
setp( ax.get_xticklabels(), visible=False)
#savefig('data/speed_missions_comparison_2013.png')
plt.show(block=True)


