#!/usr/bin/python
#Project Storm: Plot trajectories of convective systems
#import libraries
import sys
sys.path.insert(0, './src/core')
import csv, scipy
import numpy as np
from mpl_toolkits.basemap import Basemap
import  matplotlib.pyplot as plt
from liblas import file


#######################################
path_to_map_file = '/home/jhidalgocarrio/exoter/development/decos_terrain/2014_09_24_test_flight_decos_dsm.las'
path_to_map_file = 'ouput_decos.shp'
#######################################

f = file.File(path_to_map_file, mode='r')


x=[]
y=[]
z=[]

for p in f:
    x.append(p.x)
    y.append(p.y)
    z.append(p.z)

# Plot the hypocenters, colored and scaled by magnitude
mlab.points3d(x, y, z)

mlab.show()


import gdal
from mpl_toolkits.mplot3d import Axes3D

ds = gdal.Open(path_to_map_file)

import gdal
from mpl_toolkits.mplot3d import Axes3D

ds = gdal.Open('output.tiff')

dem = ds.ReadAsArray()
gt = ds.GetGeoTransform()

ds = None


fig, ax = plt.subplots(figsize=(16,8), subplot_kw={'projection': '3d'})

xres = gt[1]
yres = gt[5]

X = np.arange(gt[0], gt[0] + dem.shape[1]*xres, xres)
Y = np.arange(gt[3], gt[3] + dem.shape[0]*yres, yres)

X, Y = np.meshgrid(X, Y)

surf = ax.plot_surface(X,Y,dem, rstride=1, cstride=1, cmap=plt.cm.RdYlBu_r, vmin=0, vmax=4000, linewidth=0, antialiased=True)

ax.set_zlim(0, 60000) # to make it stand out less
ax.view_init(60,-105)

fig.colorbar(surf, shrink=0.4, aspect=20)
