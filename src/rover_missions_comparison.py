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

#################################################################################
# Data values computed at 2017
speed_mission_avg=[1.2048192771, 5.8708414873, 9.2, 19.1780821918, 200]
speed_loco_max_avg = [1.20, 92, 100.0, 100.0, 200]
speed_loco_nominal_avg = [1.20, 10, 43, 50, 200]
#################################################################################
# Plot the values
#################################################################################
matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)
#ax.set_title('Average Speed per Sol', fontsize=25, fontweight='bold')
ax.set_xlim(-1.0, 50)
ax.set_ylim(-1.0, 210)
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.spines['bottom'].set_position(('data',0))
ax.yaxis.set_ticks_position('left')
ax.spines['left'].set_position(('data',0))
#ax.grid(True)
adjust_spines(ax,['left', 'bottom'])

xposition = [5, 15, 25, 35, 45]

#Maximum average velocity
yposition = speed_loco_max_avg
plt.plot(xposition, yposition, marker='.', linestyle='--', color="lightgray", lw=4,
        label="direct driving")
plt.plot(xposition[0], yposition[0], linestyle='none', marker='s',
        color='lightgray', markersize=20)
plt.plot(xposition[1], yposition[1], linestyle='none', marker='s',
        color='lightgray', markersize=20)
plt.plot(xposition[2], yposition[2], linestyle='none', marker='s',
        color='lightgray', markersize=20)
plt.plot(xposition[3], yposition[3], linestyle='none', marker='s',
        color='lightgray', markersize=20)

plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=30)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=30)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3], yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points',
                                fontsize=30)

yposition = speed_loco_nominal_avg
plt.plot(xposition, yposition, marker='.', linestyle='-.', color="grey", lw=4,
        label="autonomous driving")
plt.plot(xposition[0], yposition[0], linestyle='none', marker='8',
        color='grey', markersize=20)
plt.plot(xposition[1], yposition[1], linestyle='none', marker='8',
        color='grey', markersize=20)
plt.plot(xposition[2], yposition[2], linestyle='none', marker='8',
        color='grey', markersize=20)
plt.plot(xposition[3], yposition[3], linestyle='none', marker='8',
        color='grey', markersize=20)


plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=30)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=30)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3], yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points',
                                fontsize=30)


#Mission average velocities
yposition = speed_mission_avg
plt.plot(xposition, yposition, marker='.', linestyle='-', color="black", lw=4,
        label="mission velocity")
plt.plot(xposition[0], yposition[0], linestyle='none', marker='D',
        color='black', markersize=20)
plt.plot(xposition[1], yposition[1], linestyle='none', marker='D',
        color='black', markersize=20)
plt.plot(xposition[2], yposition[2], linestyle='none', marker='D',
        color='black', markersize=20)
plt.plot(xposition[3], yposition[3], linestyle='none', marker='D',
        color='black', markersize=20)
plt.plot(xposition[4], yposition[4], linestyle='none', marker='D',
        color='black', markersize=20)


plt.annotate(r''+trunc(yposition[0],1), xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=30)
plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]-2.0), xycoords='data',
                                xytext=(10, -15), textcoords='offset points',
                                fontsize=30)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]),
        xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=30)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3]-2.0, yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points',
                                fontsize=30)
plt.annotate(r''+trunc(yposition[4],1), xy=(xposition[4], yposition[4]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=30)


labels = ['1996', '2003', '2011', '2020', '2025+']
plt.xticks(xposition, labels, rotation='horizontal')

plt.ylabel(r'Average velocity [$m/sol$]', fontsize=40,  fontweight='bold')
plt.legend(loc=2, prop={'size':35})
#setp( ax.get_xticklabels(), visible=False)
savefig('data/speed_missions_comparison_2017.png')
plt.show(block=True)

##############################
# Plot rover mass
##############################
matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)

#Sojourner
plt.bar(2, 1.15, 10, color="blue") #x, y and width

#MER
plt.bar(17, 18.5, 10, color="orange") #x, y and width

#MSL
plt.bar(32, 90, 10, color="yellow") #x, y and width

#ExoMars
plt.bar(47, 30, 10, color="green") #x, y and width

#SFR
plt.bar(62, 15, 10, color="firebrick") #x, y and width

ax.set_xlim(-1.0, 80)
ax.set_ylim(-1.0, 150)
#ax.grid(True)

plt.show(block=True)

##############################
# Plot mission distances
##############################
matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)

#Sojourner
plt.bar(2, 1, 100, color="blue") #x, y and width

#MER
plt.bar(152, 6, 100, color="orange") #x, y and width

#MSL
plt.bar(302, 190, 100, color="yellow") #x, y and width

#ExoMars
plt.bar(452, 40, 100, color="green") #x, y and width

#SFR
plt.bar(602, 220, 100, color="firebrick") #x, y and width

ax.set_xlim(-1.0, 800)
ax.set_ylim(-1.0, 250)
#ax.grid(True)

plt.show(block=True)


###################################
# Plot mass and mission distances
###################################
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from io import StringIO

s = StringIO(u"""\
mass     distance
1996     11.5   100
2003     185   600
2011    900  19000
2020    300  4000
2025+     150   22000""")

df = pd.read_csv(s, index_col=0, delimiter=' ', skipinitialspace=True)

matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111) # Create matplotlib axes
ax2 = ax.twinx() # Create another axes that shares the same x-axis as ax.

width = 0.4

df.mass.plot(kind='bar', color='grey', ax=ax, width=width, position=1, rot=0)
df.distance.plot(kind='bar', color='lightgray', ax=ax2, width=width, position=0,
        rot=0)

ax.set_ylabel(r'Mass [$kg$]', fontsize=40, fontweight='bold', color="black")
ax2.set_ylabel(r'Distance [$m$]', fontsize=40, fontweight='bold', color="black")
ax2.tick_params(axis='y', colors='grey')

for p in ax.patches:
    ax.annotate(str(p.get_height()), (p.get_x() * 1.005, p.get_height() *
        1.01), color="black")

for p in ax2.patches:
    ax2.annotate(str(p.get_height()), (p.get_x() * 1.005, p.get_height() *
        1.01), color="grey")

ax.set_ylim(-0.0, 1500)
ax2.set_ylim(-0.0, 25000)
ax.spines['top'].set_visible(False)
ax2.spines['top'].set_visible(False)
savefig('data/distance_and_mass_missions_comparison.png')
plt.show(block=True)



