#!/usr/bin/env python

path = '/home/javi/exoter/development/data/20140911_decos_field/20140911-1805_odometry_comparison_bis/'
#######################################
path_odometry_file = path + 'pose_odo_position.reaction_forces.0.data'

path_skid_file = path + 'pose_skid_position.0.data'

path_reference_file = path + 'pose_ref_position.0.data'
#######################################
pose_odo_orient_file = path + "pose_odo_orientation.reaction_forces.0.data"

pose_ref_orient_file = path + "pose_ref_orientation.0.data"
#######################################
decos_dem_file = '/home/javi/exoter/development/decos_terrain/decos_selected_testing_area_2_point_cloud.ply'
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
from plyfile import PlyData, PlyElement



#ExoTeR Odometry
odometry = data.ThreeData()
odometry.readData(path_odometry_file, cov=True)

# Read the odometry orientation information
odometry_orient = data.QuaternionData()
odometry_orient.readData(pose_odo_orient_file, cov=True)

#Skid Odometry
skid = data.ThreeData()
skid.readData(path_skid_file, cov=True)


#GNSS Pose
reference = data.ThreeData()
reference.readData(path_reference_file, cov=True)

# Read the reference orientation information
reference_orient = data.QuaternionData()
reference_orient.readData(pose_ref_orient_file, cov=True)

########################
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference.cov[:,0,0]))
temindex = np.asarray(temindex)

reference.delete(temindex)
odometry.delete(temindex)
skid.delete(temindex)
reference_orient.delete(temindex)
odometry_orient.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
reference.eigenValues()
odometry.eigenValues()
skid.eigenValues()
reference_orient.eigenValues()
odometry_orient.eigenValues()

##########
## PLOT ##
##########

# Terrain DEM
plydata = PlyData.read(open(decos_dem_file))

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
map_posi_align = [2.00, 8.00, 0.00]
map_orient_align = quat.quaternion.fromAngleAxis(-20.0 * np.pi/180.0, [0.0, 0.0,1.0])

#################################################
# Take the misalignment between both orientations
#################################################
misalignment = ~odometry_orient.data[1000] * reference_orient.data[1000] #~ is invert operator

##########
## PLOT ##
##########

#Position comparison X-Y plane
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

# Display the DEM
plt.rc('text', usetex=False)# activate latex text rendering
CS = plt.contour(xi, yi, zi, 25, linewidths=0.5, colors='k')
CS = plt.contourf(xi, yi, zi, 25, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())
cbar = plt.colorbar()  # draw colorbar
cbar.ax.set_ylabel('terrain elevation')
# plot data points.
plt.xlim(min(px), max(xi))
plt.ylim(min(py), max(yi))


position = np.column_stack((odometry.getAxis(0)[0::50], odometry.getAxis(1)[0::50], odometry.getAxis(2)[0::50]))
position[:] = [(misalignment * x * misalignment.conj())[1:4] for x in position ]
position[:] = [(map_orient_align * x * map_orient_align.conj())[1:4] for x in position ]
position[:] = [ x + map_posi_align for x in position ]
xposition = position[:,0]
yposition = position[:,1]
plt.rc('text', usetex=False)# activate latex text rendering
ax.plot(xposition, yposition, marker='o', linestyle='-.', label="Enhanced 3D Odometry", color=[0.3,1.0,0.4], lw=2)

xposition = odometry.getAxis(0)[0::50]# reduce number of points
yposition = odometry.getAxis(1)[0::50]
#xycov = odometry.getCov(1)[0::10]
#for i in range(0, len(xycov)):
#    cov.plot_cov_ellipse(xycov[i], pos=[xposition[i], yposition[i]], nstd=3,
#                    linewidth=2, alpha=0.5, facecolor='green', edgecolor='black')

misalignment = quat.quaternion(np.array([1.0,0.0,0.0,0.0]))
position = np.column_stack((skid.getAxis(0)[0::50], skid.getAxis(1)[0::50], skid.getAxis(2)[0::50]))
position[:] = [(misalignment * x * misalignment.conj())[1:4] for x in position ]
position[:] = [(map_orient_align * x * map_orient_align.conj())[1:4] for x in position ]
position[:] = [ x + map_posi_align for x in position ]
xposition = position[:,0]
yposition = position[:,1]
ax.plot(xposition, yposition, marker='x', linestyle='--', label="Skid Steer Odometry", color=[0,0.5,1], lw=2)


position = np.column_stack((reference.getAxis(0)[0::50],
    reference.getAxis(1)[0::50], reference.getAxis(2)[0::50]))
position[:] = [(map_orient_align * x * map_orient_align.conj())[1:4] for x in position ]
position[:] = [ x + map_posi_align for x in position ]
xposition = position[:,0]
yposition = position[:,1]
ax.plot(xposition, yposition, marker='D', linestyle='--', label="GPS Reference", color=[0.5,0,0], alpha=0.5, lw=2)
ax.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
ax.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
ax.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-40, -40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))
ax.annotate(r'End', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(-40, +40), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

#xycov = reference.getCov(1)[0::100]
#for i in range(0, len(xycov)):
#    cov.plot_cov_ellipse(xycov[i], pos=[xposition[i], yposition[i]], nstd=3,
#                    linewidth=2, alpha=0.2, facecolor=[0.4,0,0.4], edgecolor='black')

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
ax.legend(loc=1, prop={'size':30})
ax.set_axis_bgcolor('gray')
plt.axis('equal')
plt.grid(True)
plt.show(block=False)

savefig('figures/odometry_pose_decos_terrain_20140911-1805.png')

#################################################
# Take the misalignment between both orientations
#################################################
misalignment = ~odometry_orient.data[1000] * reference_orient.data[1000] #~ is invert operator

#3D Plotting values
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
fig = plt.figure()
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
ax = fig.add_subplot(111, projection='3d')

position = np.column_stack((odometry.getAxis(0)[0::50], odometry.getAxis(1)[0::50], odometry.getAxis(2)[0::50]))
position[:] = [(misalignment * x * misalignment.conj())[1:4] for x in position ]

xposition = position[:,0]
yposition = position[:,1]
zposition = position[:,2]
ax.plot(xposition, yposition, zposition, marker='o', linestyle='-.',
        label="Enhanced 3D Odometry", color=[0.3,1.0,0.4], lw=2)

position = np.column_stack((skid.getAxis(0)[0::50], skid.getAxis(1)[0::50], skid.getAxis(2)[0::50]))
position[:] = [(misalignment * x * misalignment.conj())[1:4] for x in position ]

xposition = position[:,0]
yposition = position[:,1]
zposition = position[:,2]
ax.plot(xposition, yposition, zposition, marker='^', linestyle='-',
        label="Skid Steer Odometry", color=[0,0.5,1], lw=2)

xposition = reference.getAxis(0)[0::50]
yposition = reference.getAxis(1)[0::50]
zposition = reference.getAxis(2)[0::50]
ax.plot(xposition, yposition, zposition, marker='D', linestyle='--', label="GPS Reference", color=[0.5,0,0], alpha=0.5, lw=2)

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


