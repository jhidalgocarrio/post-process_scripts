#!/usr/bin/env python

#######################################
path_reference_file_one = '/home/jhidalgocarrio/exoter/development/post-process_data/20140827_gnss_estec_lap/gnss_position.1.data'
path_reference_file_two = '/home/jhidalgocarrio/exoter/development/post-process_data/20140827_gnss_estec_lap/gnss_lat_long_quality.1.data'
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

from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

#GNSS Pose
gnss = data.ThreeData()
gnss.readData(path_reference_file_one, cov=True)
gnss.eigenValues()

#GNSS Quality
spamReader = csv.reader(open(path_reference_file_two, 'rb'), delimiter=' ', quotechar='|')
time=[]
latitude=[]
longitude=[]
quality=[]
number_satellites=[]

def qualityDic(x):
    return {
            'NO_SOLUTION' : 0,
            'AUTONOMOUS' : 1,
            'DIFFERENTIAL' : 2,
            'RTK_FIXED' : 4,
            'RTK_FLOAT' : 5,
    }.get(x, 0)

for row in spamReader:
    #print row
    time.append(float(row[0])/1000000)
    latitude.append(float(row[1]))
    longitude.append(float(row[2]))
    quality.append(qualityDic(row[3]))
    number_satellites.append(float(row[4]))


#Plot GNSS Pose
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
xposition = gnss.getAxis(0)
yposition = gnss.getAxis(1)
ax.plot(xposition, yposition, marker='^', linestyle='-', label="GNSS possition", color=[0.34,0,1], lw=2)
plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=4, prop={'size':30})
plt.show(block=False)


#3D Plot GNSS Pose
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
ax = fig.add_subplot(111, projection='3d')

xposition = gnss.getAxis(0)
yposition = gnss.getAxis(1)
zposition = gnss.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='o', linestyle='-.', label="GNSS Position", color=[0.0,0.8,0], alpha=0.5, lw=2)

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

#
#Plot the position color is quality based
#

fig, ax = plt.subplots()

xposition = np.asarray(gnss.getAxis(0))
yposition = np.asarray(gnss.getAxis(1))

# Create a colormap for red, green and blue and a norm to color
# f' < -0.5 red, f' > 0.5 blue, and the rest green
cmap = ListedColormap(['b', 'g', 'r', 'm', 'c', 'k'])
norm = BoundaryNorm([-1, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5], cmap.N)

points = np.array([xposition, yposition]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

lc = LineCollection(segments, cmap=cmap, norm=norm)
lc.set_array(np.asarray(quality))
lc.set_linewidth(3)

ax.add_collection(lc)

image_data = np.clip([xposition, yposition], -1, 1)
cax = ax.imshow(image_data, interpolation='nearest', cmap=cmap)

cbar = fig.colorbar(cax, ticks=[-1, 0, 1])# orientation='horizontal')
cbar.ax.set_xticklabels(['Low', 'Medium', 'High'])# horizontal colorbar

#ax.legend(handles=[blue_patch] )
plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
plt.xlim(xposition.min(), xposition.max())
plt.ylim(yposition.min(), yposition.max())
plt.show(block=False)

#
#Plot the position color is number of satellites track (continuous color map)
#
fig, ax = plt.subplots()

number_satellites = np.asarray(number_satellites)
xposition = np.asarray(gnss.getAxis(0))
yposition = np.asarray(gnss.getAxis(1))

points = np.array([xposition, yposition]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

lc = LineCollection(segments, cmap=plt.get_cmap('Blues'), norm=plt.Normalize(0.00, number_satellites.max()))
lc.set_array(number_satellites)
lc.set_linewidth(3)

ax.add_collection(lc)

image_data = np.clip([xposition, yposition], number_satellites.min(), number_satellites.max())
cax = ax.imshow(image_data, interpolation='nearest', cmap=plt.get_cmap('Blues'))

cbar = fig.colorbar(cax, ticks=[number_satellites.min(), number_satellites.mean(), number_satellites.max()])# orientation='horizontal')
#cbar.ax.set_xticklabels(['Low', 'Medium', 'High'])# horizontal colorbar
ax.set_title('GNSS Test - Color based on number of satellites tracked')

#ax.legend(handles=[blue_patch] )
plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
plt.grid(True)
plt.xlim(xposition.min(), xposition.max())
plt.ylim(yposition.min(), yposition.max())
plt.show(block=False)



