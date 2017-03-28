#!/usr/bin/env python

path ='~/npi/data/20140911_decos_field/20140911-1805_odometry_comparison_bis/'
#######################################
path_odometry_file = path + 'pose_odo_position.reaction_forces.0.data'

path_skid_file = path + 'pose_skid_position.0.data'

path_reference_file = path + 'pose_ref_position.0.data'
#######################################
pose_odo_orient_file = path + "pose_odo_orientation.reaction_forces.0.data"

pose_ref_orient_file = path + "pose_ref_orientation.0.data"
#######################################
decos_dem_file ='~/npi/documentation/decos_terrain/decos_selected_testing_zoom_area_point_cloud.ply'
#######################################


import sys
sys.path.insert(0, './src/core')
import os
import csv
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov
from plyfile import PlyData, PlyElement
import scipy
from numpy import linalg as la

import pandas as pandas
import datetime
matplotlib.style.use('ggplot') #in matplotlib >= 1.5.1

def dateparse (time_in_microsecs):
    return datetime.datetime.fromtimestamp(float(time_in_microsecs) * 1e-06)

##########################################################################
# PLOTTING FUNCTION
##########################################################################
def decos_odometry_dem_figure(fig_num, dem_file,
        threed_odometry_trajectory, skid_odometry_trajectory,
        reference_trajectory, mis_orient = quat.quaternion(np.array([1.0,0.0,0.0,0.0])),
        mis_position = [0.00, 0.00, 0.00]):

    ########################
    # Load Terrain DEM
    ########################
    import os
    plydata = PlyData.read(open(os.path.expanduser(dem_file)))

    vertex = plydata['vertex'].data

    [dem_px, dem_py, dem_pz] = (vertex[t] for t in ('x', 'y', 'z'))

    # define grid.
    npts=100
    dem_xi = np.linspace(min(dem_px), max(dem_px), npts)
    dem_yi = np.linspace(min(dem_py), max(dem_py), npts)

    # grid the data.
    dem_zi = griddata(dem_px, dem_py, dem_pz, dem_xi, dem_yi, interp='linear')

    #################################################
    # Misalignment to the map                       #
    #################################################
    map_posi_align = [2.00, 9.00, 0.00]
    map_orient_align = quat.quaternion.fromAngleAxis(-20.0 * np.pi/180.0, [0.0, 0.0,1.0])

    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
    fig = plt.figure(fig_num, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.add_subplot(111)
    #fig, ax = plt.subplots()

    # Display the DEM
    plt.rc('text', usetex=False)# activate latex text rendering
    CS = plt.contour(dem_xi, dem_yi, dem_zi, 20, linewidths=0.5, colors='k')
    CS = plt.contourf(dem_xi, dem_yi, dem_zi, 20, cmap=plt.cm.gray, vmax=abs(dem_zi).max(), vmin=-abs(dem_zi).max())
    #plt.clabel(CS, inline=1, fontsize=10) # number in the contour line

    # plot data points.
    plt.xlim(min(dem_px), max(dem_xi))
    plt.ylim(min(dem_py), max(dem_yi))

    # Color bar of the dem
    cbar = plt.colorbar()  # draw colorbar
    cbar.ax.set_ylabel(r' terrain elevation [$m$]', fontsize=25, fontweight='bold')

    # Plot 3d odometry trajectory
    t_odo = np.column_stack((threed_odometry_trajectory[:,0][0::20],
        threed_odometry_trajectory[:,1][0::20],
        threed_odometry_trajectory[:,2][0::20]))
    t_odo[:] = [(mis_orient * i * mis_orient.conj())[1:4] for i in t_odo]
    t_odo[:] = [ i + mis_position for i in t_odo ]
    t_odo[:] = [(map_orient_align * i * map_orient_align.conj())[1:4] for i in t_odo ]
    t_odo[:] = [ i + map_posi_align for i in t_odo ]
    x = t_odo[:,0]
    y = t_odo[:,1]
    ax.plot(x, y, marker='o', linestyle='-.', label="3d odometry", color=[0.3,1.0,0.4], lw=2)

    # Plot skid odometry
    skid = np.column_stack((skid_odometry_trajectory[:,0][0::20],
        skid_odometry_trajectory[:,1][0::20],
        skid_odometry_trajectory[:,2][0::20]))
    skid[:] = [(mis_orient * i * mis_orient.conj())[1:4] for i in skid]
    skid[:] = [ i + mis_position for i in skid ]
    skid[:] = [(map_orient_align * i * map_orient_align.conj())[1:4] for i in skid ]
    skid[:] = [ i + map_posi_align for i in skid ]
    x = skid[:,0]
    y = skid[:,1]
    ax.plot(x, y, marker='x', linestyle='--', label="skid odometry", color=[0,0.5,1], lw=2)

    # Display Ground Truth trajectory
    ref = np.column_stack((reference_trajectory[:,0][0::20], reference_trajectory[:,1][0::20], reference_trajectory[:,2][0::20]))
    ref[:] = [(map_orient_align * i * map_orient_align.conj())[1:4] for i in ref ]
    ref[:] = [ i + map_posi_align for i in ref ]
    x = ref[:,0]
    y = ref[:,1]
    ax.plot(x, y, marker='D', linestyle='--', label="reference trajectory", color=[0.5,0,0], alpha=0.5, lw=2)


    import os
    from matplotlib.cbook import get_sample_data
    from matplotlib._png import read_png
    import matplotlib.image as image
    from scipy import ndimage
    from matplotlib.offsetbox import OffsetImage, AnnotationBbox
    fn = get_sample_data(os.getcwd()+"/data/img/exoter.png", asfileobj=False)
    exoter = image.imread(fn)
    exoter = ndimage.rotate(exoter, -120)
    imexoter = OffsetImage(exoter, zoom=0.3)


    ab = AnnotationBbox(imexoter, xy=(x[0], y[0]),
                    xybox=None,
                    xycoords='data',
                    boxcoords="offset points",
                    frameon=False)

    ax.annotate(r'ExoTeR', xy=(x[0], y[0]), xycoords='data',
                            xytext=(-20, -35), textcoords='offset points', fontsize=16,
                            #arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0)
                            )

    ax.annotate(r'Start', xy=(x[0], y[0]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points', fontsize=16,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[0], y[0], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)

    #ax.arrow(x[0], y[0], x[13]-x[0], y[13]-y[0], width=0.02, head_width=0.1,
    #        head_length=0.05, fc='k', ec='k', zorder=104)

    # End sign
    ax.annotate(r'End', xy=(x[x.shape[0]-1], y[y.shape[0]-1]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points', fontsize=16,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[x.shape[0]-1], y[y.shape[0]-1], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)

    ax.add_artist(ab)

    plt.xlabel(r'X [$m$]', fontsize=15, fontweight='bold')
    plt.ylabel(r'Y [$m$]', fontsize=15, fontweight='bold')
    ax.legend(loc=2, prop={'size':15})
    #leg = ax.legend(loc=1, prop={'size':30}, fancybox=True)
    #leg.get_frame().set_alpha(0.5)
    #plt.axis('equal')
    plt.grid(True)
    fig.savefig("decos_odometry_comparison_20140911-1805.png", dpi=fig.dpi)
    plt.show(block=True)

##########################################################################
# READ THE VALUES IN PANDAS
##########################################################################

#ExoTeR Odometry
threed_odometry = pandas.read_csv(os.path.expanduser(path_odometry_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#Skid Odometry
skid_odometry = pandas.read_csv(os.path.expanduser(path_skid_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

#Vicon Pose
reference = pandas.read_csv(os.path.expanduser(path_reference_file), sep=" ", parse_dates=True,
    date_parser=dateparse , index_col='time',
    names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
        'cov_zx', 'cov_zy', 'cov_zz'], header=None)

# Read the odometry orientation information
odometry_orient = data.QuaternionData()
odometry_orient.readData(pose_odo_orient_file, cov=True)

# Read the reference orientation information
reference_orient = data.QuaternionData()
reference_orient.readData(pose_ref_orient_file, cov=True)

##########################################################################
#mask = pandas.notnull(reference.x) & pandas.notnull(reference.y) & pandas.notnull(reference.z)
#reference = reference[mask]

#################
## RE-SAMPLE   ##
#################
#resampling_time = '1s'
#threed_odometry = threed_odometry.resample(resampling_time).mean()
#skid_odometry = skid_odometry.resample(resampling_time).mean()
#reference = reference.resample(resampling_time).mean()

##########################################################################
#rotate and translate the 3d odometry trajectory wrt the world frame
##########################################################################
threed_odometry_position = np.column_stack((threed_odometry.x.values, threed_odometry.y.values,  threed_odometry.z.values ))

##########################################################################
#rotate and translate the skid odometry trajectory wrt the world frame
##########################################################################
skid_odometry_position = np.column_stack((skid_odometry.x.values, skid_odometry.y.values,  skid_odometry.z.values ))

##########################################################################
#rotate and translate the reference trajectory wrt the world frame
##########################################################################
reference_position = np.column_stack((reference.x.values, reference.y.values,  reference.z.values ))

#################################################
# Take the misalignment between both orientations
#################################################
mis_orient = quat.quaternion.fromAngleAxis(5.0 * np.pi/180.0, [0.0, 0.0, 1.0])
mis_position = [0.0, 0.0, 0.00]

############
### PLOT ###
############
decos_odometry_dem_figure(1, decos_dem_file, threed_odometry_position, skid_odometry_position, reference_position, mis_orient, mis_position)

#################################################
# 3D PLOT
#################################################
import os
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d

plydata = PlyData.read(open(os.path.expanduser(decos_dem_file)))

vertex = plydata['vertex'].data

[px, py, pz] = (vertex[t] for t in ('x', 'y', 'z'))

# define grid.
npts=500
xi = np.linspace(min(px), max(px), npts)
yi = np.linspace(min(py), max(py), npts)

# grid the data.
zi = griddata(px, py, pz, xi, yi, interp='linear')

#################################################
# Misalignment to the map                       #
#################################################
map_posi_align = [2.00, 9.00, 0.00]
map_orient_align = quat.quaternion.fromAngleAxis(-20.0 * np.pi/180.0, [0.0, 0.0,1.0])

fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
ax = fig.add_subplot(111, projection='3d')
x, y = np.meshgrid(xi, yi)
ax.plot_surface(x, y, zi, rstride=2, cstride=2, alpha=1.0, linewidth=0.3, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())
cset = ax.contour(x, y, zi, zdir='z', cmap=plt.cm.coolwarm,
        vmax=abs(zi).max(), vmin=-abs(zi).max(), linewidth=4.0)

plt.rc('text', usetex=False)# activate latex text rendering

# Plot 3d odometry trajectory
position = np.column_stack((threed_odometry_position[:,0][0::50], threed_odometry_position[:,1][0::50], threed_odometry_position[:,2][0::50]))
position[:] = [(mis_orient * i * mis_orient.conj())[1:4] for i in position]
position[:] = [ i + mis_position for i in position ]
position[:] = [(map_orient_align * i * map_orient_align.conj())[1:4] for i in position ]
position[:] = [ i + map_posi_align for i in position ]

xposition = position[:,0]
yposition = position[:,1]
zposition = position[:,2]
ax.plot(xposition, yposition, zposition, marker='o', linestyle='-.', label="3d odometry", color=[0.3,1.0,0.4], lw=2)

# Plot skid odometry trajectory
position = np.column_stack((skid_odometry_position[:,0][0::50], skid_odometry_position[:,1][0::50], skid_odometry_position[:,2][0::50]))
position[:] = [(mis_orient * i * mis_orient.conj())[1:4] for i in position]
position[:] = [ i + mis_position for i in position ]
position[:] = [(map_orient_align * i * map_orient_align.conj())[1:4] for i in position ]
position[:] = [ i + map_posi_align for i in position ]

xposition = position[:,0]
yposition = position[:,1]
zposition = position[:,2]
ax.plot(xposition, yposition, zposition, marker='^', linestyle='-', label="skid odometry", color=[0,0.5,1], lw=2)

# Plot skid odometry trajectory
position = np.column_stack((reference_position[:,0][0::50], reference_position[:,1][0::50], reference_position[:,2][0::50]))
position[:] = [(map_orient_align * i * map_orient_align.conj())[1:4] for i in position ]
position[:] = [ i + map_posi_align for i in position ]
xposition = position[:,0]
yposition = position[:,1]
zposition = position[:,2]
ax.plot(xposition, yposition, zposition, marker='D', linestyle='--', label="reference trajectory", color=[0.5,0,0], alpha=0.5, lw=2)

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
plt.show(block=True)


