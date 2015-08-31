#!/usr/bin/env python

path = '/home/javi/exoter/development/data/20140723_pink_odometry/20140723-1826/'

#######################################
add_nodes_to_envire_file = path + 'envire_add_new_nodes.0.data'

remove_nodes_to_envire_file = path + 'envire_remove_existing_nodes.0.data'
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


from collections import defaultdict

### ADD NODES - READ THE FILE
add_nodes_list = []
for row in csv.reader(open(add_nodes_to_envire_file, 'r'), delimiter=' ', quotechar='|'):
    add_nodes_list.append([float(row[0]), float(row[1])/1000000.00])


add_nodes_hash = defaultdict(list)

for k, v in add_nodes_list:
    add_nodes_hash[k].append(v)


add_nodes = dict((k, tuple(v)) for k, v in add_nodes_hash.iteritems())

add_nodes_array=[]

for i, v in add_nodes_hash.iteritems():
    add_nodes_array.append([i, np.mean(v), np.std(v)])

add_nodes_array = np.array(add_nodes_array)
add_nodes_array = np.sort(add_nodes_array, axis=0)

# Position comparison versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
ax.errorbar(add_nodes_array[:,0], add_nodes_array[:,1]*1000.00, yerr=add_nodes_array[:,2]*1000.00,  fmt='--o', ecolor='r', capthick=2, lw=2.0)

plt.xlabel(r'Nodes [$number$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Time [$ms$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
#plt.axes().set_aspect('equal', 'datalim')
ax.set_title('Envire Graph - Add Nodes')
plt.show(block=False)



### REMOVE NODES - READ THE FILE
remove_nodes_list = []
for row in csv.reader(open(remove_nodes_to_envire_file, 'r'), delimiter=' ', quotechar='|'):
    remove_nodes_list.append([float(row[0]), float(row[1])/1000000.00])

remove_nodes_hash = defaultdict(list)

for k, v in remove_nodes_list:
    remove_nodes_hash[k].append(v)

remove_nodes_array=[]

for i, v in remove_nodes_hash.iteritems():
    remove_nodes_array.append([i, np.mean(v), np.std(v)])

remove_nodes_array = np.array(remove_nodes_array)
remove_nodes_array = np.sort(remove_nodes_array, axis=0)

# Position comparison versus time
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
ax.errorbar(remove_nodes_array[:,0], remove_nodes_array[:,1]*1000.00, yerr=remove_nodes_array[:,2]*1000.00,  fmt='--o', ecolor='r', capthick=2, lw=2.0)

plt.xlabel(r'Nodes [$number$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Time [$ms$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
#plt.axes().set_aspect('equal', 'datalim')
ax.set_title('Envire Graph - Remove Nodes')
plt.show(block=False)




