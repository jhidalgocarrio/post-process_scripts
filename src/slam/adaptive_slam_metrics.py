#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2017-06-20 16:35:56
import numpy as np
from pylab import *
import matplotlib.pyplot as plt

matplotlib.style.use('classic') #in matplotlib >= 1.5.1

wo_adaptivity = np.array([100.0, 100.0, 100.0])
w10_adaptivity = np.array([46.0, 64.4, 48.6])
w25_adaptivity = np.array([27.0, 27.9, 22.04])
w100_adaptivity = np.array([18.3, 19.9, 19.95])

#####################################
# Adaptivity mean and std on box plots
#####################################
matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(28, 16))
labels = ['0%', '10%', '25%', '100%']
boxprops = dict(linestyle='-', linewidth=4, color='k')
medianprops = dict(linestyle='-', linewidth=4, color='k')

data = [wo_adaptivity, w10_adaptivity, w25_adaptivity, w100_adaptivity]

bp = ax.boxplot(data,
        vert=True,  # vertical box alignment
        patch_artist=True,  # fill with color
        showfliers=False,
        showmeans=True,
        boxprops=boxprops,
        medianprops=medianprops,
        labels=labels)  # will be used to label x-ticks

for bplot in bp['boxes']:
    bplot.set_facecolor('lightgrey')

for bplot in bp['whiskers']:
    bplot.set_linewidth(4)

for bplot in bp['caps']:
    bplot.set_linewidth(4)

for line in bp['means']:
    # get position data for median line
    x, y = line.get_xydata()[0] # top of median line
    # overlay median value
    text(x, y, '        %.1f' % y,
         horizontalalignment='left') # draw above, centered

ax.set_ylabel('frames percentage reduction [$\%$]', fontsize=40, fontweight='bold', color="black")
ax.set_xlabel('adaptivity', fontsize=40, fontweight='bold', color="black")

#plt.subplots_adjust(left=0.075, right=0.95, top=0.9, bottom=0.25)

# Add a horizontal grid to the plot, but make it very light in color
# so we can use it for reading data values but not be distracting
ax.yaxis.grid(True, linestyle='-', which='major', color='grey', alpha=0.5)
# Hide these grid behind plot objects
ax.set_axisbelow(True)

plt.show(block=True)

savefig('adaptive_slam_boxplots_numbers.pdf')
#####################################
wo_error = np.array([0.0, 0.0, 0.0])
w10_error = np.array([0.02, 0.14, 0.09])
w25_error = np.array([0.12, 0.26, 0.11])
w100_error = np.array([0.92, 1.12, 0.51])

#####################################
# Adaptivity mean and std on box plots
#####################################
matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(28, 16))
labels = ['0%', '10%', '25%', '100%']
boxprops = dict(linestyle='-', linewidth=4, color='k')
medianprops = dict(linestyle='-', linewidth=4, color='k')

data = [wo_error, w10_error, w25_error, w100_error]

bp = ax.boxplot(data,
        vert=True,  # vertical box alignment
        patch_artist=True,  # fill with color
        showfliers=False,
        showmeans=True,
        boxprops=boxprops,
        medianprops=medianprops,
        labels=labels)  # will be used to label x-ticks

for bplot in bp['boxes']:
    bplot.set_facecolor('lightgrey')

for bplot in bp['whiskers']:
    bplot.set_linewidth(4)

for bplot in bp['caps']:
    bplot.set_linewidth(4)

for line in bp['means']:
    # get position data for median line
    x, y = line.get_xydata()[0] # top of median line
    # overlay median value
    text(x, y, '        %.1f' % y,
         horizontalalignment='left') # draw above, centered

ax.set_ylabel('error percentage increase [$\%$]', fontsize=40, fontweight='bold', color="black")
ax.set_xlabel('adaptivity', fontsize=40, fontweight='bold', color="black")

#plt.subplots_adjust(left=0.075, right=0.95, top=0.9, bottom=0.25)

# Add a horizontal grid to the plot, but make it very light in color
# so we can use it for reading data values but not be distracting
ax.yaxis.grid(True, linestyle='-', which='major', color='grey', alpha=0.5)
# Hide these grid behind plot objects
ax.set_axisbelow(True)

plt.show(block=True)

savefig('error_slam_boxplots_numbers.pdf')
#####################################

data_mean = np.array([wo_adaptivity.mean(), w10_adaptivity.mean(),
    w25_adaptivity.mean(), w100_adaptivity.mean()])/100.0

data_std = np.array([wo_adaptivity.std(), w10_adaptivity.std(),
    w25_adaptivity.std(), w100_adaptivity.std()])/100.0

exomars_distance = 50 #50 meter per sol in autonomous navigation

nominal_velocity = 0.02 # 2cm/sec

nav_image_time = 102 #1.7 min

loc_image_time = 90 #1.5 min

locomotion_time = exomars_distance / nominal_velocity

nav_per_frame = 2.0 #distance between navigation frames

loc_per_frame = 0.33 #distance between localization frames

number_frames = exomars_distance/loc_per_frame #frames per traverse

nav_time = exomars_distance/nav_per_frame * nav_image_time

data_time = np.array([(data_mean[0]*number_frames) * loc_image_time,
                    (data_mean[1]*number_frames) * loc_image_time,
                    (data_mean[2]*number_frames) * loc_image_time,
                    (data_mean[3]*number_frames) * loc_image_time])

data_time = data_time + locomotion_time + nav_time

nav_hours_per_sol = 3600 * 2.25 #2.25 hours per sol

meter_per_sol = (exomars_distance * nav_hours_per_sol) / data_time
#####################################
# Meter per sol with adaptive SLAM
#####################################
def adjust_spines(ax,spines):
    for loc, spine in ax.spines.items():
        if loc in spines:
            spine.set_position(('outward',10)) # outward by 10 points
            spine.set_smart_bounds(True)
        #else:
            #spine.set_color('none') # don't draw spine

    # turn off ticks where there is no spine
    if 'left' in spines:
        ax.yaxis.set_ticks_position('left')
    else:
        # no yaxis ticks
        ax.yaxis.set_ticks([0, 10, 20, 30, 40, 50, 60])

    if 'bottom' in spines:
        ax.xaxis.set_ticks_position('bottom')
    else:
        # no xaxis ticks
        ax.xaxis.set_ticks([])

def trunc(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    return ('%.*f' % (n + 1, f))[:-1]
#####################################
matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)
#ax.set_title('Average Speed per Sol', fontsize=25, fontweight='bold')
ax.set_xlim(-1.0, 40)
ax.set_ylim(-1.0, 58)
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.spines['bottom'].set_position(('data',0))
adjust_spines(ax,['bottom'])
ax.spines['left'].set_visible(True)

xposition = [5, 15, 25, 37]

# Autonomous driving velocity with adaptive slam
yposition = meter_per_sol
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

plt.annotate(r''+trunc(yposition[0],1), xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=40)

plt.annotate(r''+trunc(yposition[1],1), xy=(xposition[1], yposition[1]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=40)
plt.annotate(r''+trunc(yposition[2],1), xy=(xposition[2], yposition[2]), xycoords='data',
                                xytext=(-10, 10), textcoords='offset points',
                                fontsize=40)
plt.annotate(r''+trunc(yposition[3],1), xy=(xposition[3], yposition[3]), xycoords='data',
                                xytext=(-20, 10), textcoords='offset points',
                                fontsize=40)


labels = ['0%', '10%', '25%', '100%']
plt.xticks(xposition, labels, rotation='horizontal')

plt.ylabel(r'average velocity [$m/sol$]', fontsize=40,  fontweight='bold')
plt.xlabel(r'adaptivity', fontsize=40,  fontweight='bold')
plt.legend(loc=2, prop={'size':35})
plt.show(block=True)

