#!/usr/bin/python
#Project Storm: Plot trajectories of convective systems
#import libraries
import sys
sys.path.insert(0, './src/core')
import csv, scipy
import numpy as np
import  matplotlib.pyplot as plt
import simplekml

#######################################
#path_reference_file_two = '/home/jhidalgocarrio/exoter/development/post-process_data/20140827_gnss_estec_lap/gnss_lat_long_quality.1.data'
path_reference_file_two = '/home/jhidalgocarrio/exoter/development/post-process_data/20140911_decos_field/20140911-1805/gnss_lat_long_quality.0.data'
#######################################

spamReader = csv.reader(open(path_reference_file_two, 'rb'), delimiter=' ', quotechar='|')
time=[]
latitude=[]
longitude=[]
path=[]
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
    path.append((float(row[2]), float(row[1])))
    quality.append(qualityDic(row[3]))
    number_satellites.append(float(row[4]))

latitude = np.asarray(latitude)
longitude = np.asarray(longitude)

kml = simplekml.Kml()

lin = kml.newlinestring(name="GNSS ESTEC Test", description="ARL Pitukos Trajectory", coords=path)

lin.style.linestyle.color = 'ff0000ff'  # Red
lin.style.linestyle.width= 10  # 10 pixels

#kml.save("gnss_estec_tour.kml")
kml.save("gnss_decos_20140911-1805.kml")




