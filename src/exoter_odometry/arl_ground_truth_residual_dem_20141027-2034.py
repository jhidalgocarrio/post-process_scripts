path='/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034_threed_odometry_with_gp/'
#######################################
path_odometry_file = path + 'pose_odo_position.0.data'

path_odometry_velocity_file = path + 'delta_pose_odo_velocity.0.data'

path_reference_file = path + 'pose_ref_position.0.data'

path_reference_velocity_file = path + 'delta_pose_ref_velocity.0.data'

path_navigation_orientation_file = path + 'pose_world_to_navigation_orientation.0.data'

path_navigation_position_file = path + 'pose_world_to_navigation_position.0.data'

#######################################
esa_arl_dem_file = '/home/javi/exoter/development/esa_terrain_lab/DEMclean.ply'
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
from plyfile import PlyData, PlyElement
import scipy
from numpy import linalg as la

#ExoTeR Odometry
odometry = data.ThreeData()
odometry.readData(path_odometry_file, cov=True)

# Odometry Robot Velocity
odometry_velocity = data.ThreeData()
odometry_velocity.readData(path_odometry_velocity_file, cov=True)

#Reference Pose
reference = data.ThreeData()
reference.readData(path_reference_file, cov=True)

#Reference Velocity
reference_velocity = data.ThreeData()
reference_velocity.readData(path_reference_velocity_file, cov=True)

#World to Navigation Pose
navigation_orient = data.QuaternionData()
navigation_orient.readData(path_navigation_orientation_file, cov=False)
navigation_position = data.ThreeData()
navigation_position.readData(path_navigation_position_file, cov=False)


########################
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference.data[:,0]))
temindex = np.asarray(temindex)
odometry.delete(temindex)
reference.delete(temindex)

temindex = np.where(np.isnan(reference_velocity.data[:,0]))
temindex = np.asarray(temindex)
reference_velocity.delete(temindex)
odometry_velocity.delete(temindex)

temindex = np.where(fabs(reference_velocity.data) > 0.60)
temindex = np.asarray(temindex[0])
reference_velocity.delete(temindex)
odometry_velocity.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
odometry.covSymmetry()
odometry.eigenValues()
reference.covSymmetry()
reference.eigenValues()
odometry_velocity.covSymmetry()
odometry_velocity.eigenValues()
reference_velocity.covSymmetry()
reference_velocity.eigenValues()

# Terrain DEM
plydata = PlyData.read(open(esa_arl_dem_file))

vertex = plydata['vertex'].data

[px, py, pz] = (vertex[t] for t in ('x', 'y', 'z'))

# define grid.
npts=100
xi = np.linspace(min(px), max(px), npts)
yi = np.linspace(min(py), max(py), npts)

# grid the data.
zi = griddata(px, py, pz, xi, yi, interp='linear')

########################
## CONVOLUTE FILTER   ##
########################
sampling_frequency = 1.0/mean(odometry_velocity.delta[0:100])
size_block = 1 * sampling_frequency
window = np.ones(size_block)
window /= sum(window)

# Display Reference trajectory
#rotate and translate the trajectory wrt the world frame
position = np.column_stack((reference.getAxis(0), reference.getAxis(1),  reference.getAxis(2)))
position[:] = [navigation_orient.data[0].rot(x) +  navigation_position.data[0] for x in position]

# Reference velocity in body frame
rvelocity = np.column_stack((
            np.convolve(reference_velocity.getAxis(0), window, mode='same'),
            np.convolve(reference_velocity.getAxis(1), window, mode='same'),
            np.convolve(reference_velocity.getAxis(2), window, mode='same')
            ))

# Odometry velocity in body frame
ovelocity = np.column_stack((
            np.convolve(odometry_velocity.getAxis(0), window, mode='same'),
            np.convolve(odometry_velocity.getAxis(1), window, mode='same'),
            np.convolve(odometry_velocity.getAxis(2), window, mode='same')
            ))

length = min (rvelocity.shape[0], ovelocity.shape[0])
standard_deviation = np.absolute(ovelocity[0:length,:]-rvelocity[0:length,:])

###################################
## SPLIT INPUT TEST  (OPTIONAL)  ##
###################################
number_blocks = int(len(reference_velocity.delta)/size_block)

# Split in block to reduce the samples
position, positionstd = data.input_reduction(position, number_blocks)
rvelocity, rvelocitystd = data.input_reduction(rvelocity, number_blocks)
ovelocity, ovelocitystd = data.input_reduction(ovelocity, number_blocks)
standard_deviation, standard_deviationstd = data.input_reduction(standard_deviation, number_blocks)

############
### PLOT ###
############
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(3)
ax = fig.add_subplot(111)

# Display the DEM
plt.rc('text', usetex=False)# activate latex text rendering
CS = plt.contour(xi, yi, zi, 15, linewidths=0.5, colors='k')
CS = plt.contourf(xi, yi, zi, 15, cmap=plt.cm.gray, vmax=abs(zi).max(), vmin=-abs(zi).max())
# plot data points.
plt.xlim(min(px), max(xi))
plt.ylim(min(py), max(yi))


# Display Ground Truth trajectory
x = position[:,0]
y = position[:,1]
sd = la.norm(standard_deviation, axis=1)
points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.colors import LinearSegmentedColormap as lscm
cmap = plt.get_cmap('Reds')
#cmap = lscm.from_list('temp', colors)
norm = plt.Normalize(min(sd), max(sd))
lc = LineCollection(segments, cmap=cmap, norm=norm)
lc.set_array(sd)
lc.set_linewidth(40)
lc.set_alpha(0.8)
plt.gca().add_collection(lc)


#color bar of the covariamve
#cbaxes = fig.add_axes([0.8, 0.1, 0.03, 0.8]) 
h_cbar = plt.colorbar(lc)#, orientation='horizontal')
h_cbar.ax.set_ylabel(r' ground truth residual [$m/s$]')

# Color bar of the dem
cbar = plt.colorbar()  # draw colorbar
cbar.ax.set_ylabel(r' terrain elevation[$m$]')

#Q = ax.plot(x, y, marker='o', linestyle='-', color=[0.3,0.2,0.4], alpha=0.5, lw=40)

import os
from matplotlib.cbook import get_sample_data
from matplotlib._png import read_png
import matplotlib.image as image
from scipy import ndimage
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
fn = get_sample_data(os.getcwd()+"/data/img/exoter.png", asfileobj=False)
exoter = image.imread(fn)
exoter = ndimage.rotate(exoter, 180)
imexoter = OffsetImage(exoter, zoom=0.3)


ab = AnnotationBbox(imexoter, xy=(x[0], y[0]),
                        xybox=None,
                        xycoords='data',
                        boxcoords="offset points",
                        frameon=False)

ax.annotate("ExoTeR", xy=(x[0], y[0]), xycoords='data',
                                xytext=(-40, 45), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", lw=2.0))

ax.add_artist(ab)

plt.xlabel(r'X [$m$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Y [$m$]', fontsize=35, fontweight='bold')
#plt.axis('equal')
plt.grid(True)
#ax.legend(handles=[exoter], loc=1, prop={'size':30})
plt.show(block=False)


