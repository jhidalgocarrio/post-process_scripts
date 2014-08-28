#!/usr/bin/python
#Project Storm: Plot trajectories of convective systems
#import libraries
import sys
sys.path.insert(0, './src/core')
import csv, scipy
import numpy as np
from mpl_toolkits.basemap import Basemap
import  matplotlib.pyplot as plt


#######################################
path_reference_file_two = '/home/jhidalgocarrio/exoter/development/post-process_data/20140827_gnss_estec_lap/gnss_lat_long_quality.1.data'
#######################################

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

latitude = np.asarray(latitude)
longitude = np.asarray(longitude)


# Plot a map for Mexico

m = Basemap(projection='cyl', llcrnrlat=latitude.min()-4.0, urcrnrlat=latitude.max()+4.0, llcrnrlon=longitude.min()-4.0, urcrnrlon=longitude.max()+4.0, resolution='f', area_thresh=100.)
m.bluemarble()
m.drawcoastlines(linewidth=0.5)
m.drawcountries(linewidth=0.5)
m.drawstates(linewidth=0.5)
m.fillcontinents(color='coral')
m.drawmapboundary()

#Draw parallels and meridians

m.drawparallels(np.arange(10.,35.,5.))
m.drawmeridians(np.arange(-120.,-80.,10.))
m.drawmapboundary(fill_color='aqua')


#Convert latitude and longitude to coordinates X and Y

x, y = m(longitude, latitude)

#Plot the points on the map
plt.plot(x,y,'ro-')
#lg = plt.legend()
#lg.get_frame().set_facecolor('grey')
plt.show()
