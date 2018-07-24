#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2018-07-23 14:10:35
#######################################
path='~/waste_shark/experiments/20180720-1644_gt_maritime_hall/'
#######################################
path_reference_position_file = path + '2018-07-20-16-44-13.bag'
#######################################

import sys
import os
sys.path.insert(0, './src/core')
sys.path.insert(0, './src/ros_numpy/src')
import quaternion as quat
from pylab import *
import rosbag
from std_msgs.msg import Int32, String
import numpy as np
import matplotlib.pyplot as plt
import scipy
import pandas as pandas
import datetime
matplotlib.style.use('classic') #in matplotlib >= 1.5.1


#import rosbag_pandas
#df = rosbag_pandas.bag_to_dataframe(os.path.expanduser(path_reference_position_file)) #awesome data processing

############################
# Process the ROS bag logs
############################
bag = rosbag.Bag(os.path.expanduser(path_reference_position_file))
pose_time = []
pose_pos = []
pose_orient = []
for topic, msg, t in bag.read_messages(topics=['/tf']):
    if msg.transforms[0].child_frame_id == 'tag':
        print "-----"
        print msg
        print "-----"
        print t
        pose_time.append(t.to_time())
        position = msg.transforms[0].transform.translation
        orientation = msg.transforms[0].transform.rotation
        pose_pos.append(np.array([position.x, position.y, position.z]))
        pose_orient.append(np.array([orientation.w, orientation.x, orientation.y, orientation.z]))


pose_time = np.asarray(pose_time)
pose_pos = np.asarray(pose_pos)
pose_orient = np.asarray(pose_orient)

############################
# Convert to pandas
############################
reference_position = pandas.DataFrame(data=pose_pos, index=[datetime.datetime.fromtimestamp(x) for x in pose_time], columns=['x', 'y', 'z'])
reference_orient = pandas.DataFrame(data=pose_orient, index=[datetime.datetime.fromtimestamp(x) for x in pose_time], columns=['w', 'x', 'y', 'z'])



##########################################################################
# PLOTTING FUNCTION
##########################################################################
def maritime_hall_figure(fig_num, dem_file, reference_trajectory,
        slam_trajectory,
        mis_orient = quat.quaternion(np.array([1.0,0.0,0.0,0.0])),
        mis_position = [0.00, 0.00, 0.00]):

    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
    fig = plt.figure(fig_num, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.add_subplot(111)


    # Display Ground Truth trajectory
    from numpy import linalg as la
    ref = np.column_stack((reference_trajectory[:,0][0::10], reference_trajectory[:,1][0::10], reference_trajectory[:,2][0::10]))
    ref[:] = [(mis_orient * i * mis_orient.conj())[1:4] for i in ref ]
    ref[:] = [ i + mis_position for i in ref ]
    x = ref[:,0]
    y = ref[:,1]
    ax.plot(x, y, marker='D', linestyle='-', lw=2, alpha=0.3, color=[1.0, 1.0, 0.0],
            label='ground truth', zorder=80)

    # Annotations
    import os
    from matplotlib.cbook import get_sample_data
    from matplotlib._png import read_png
    import matplotlib.image as image
    from matplotlib.offsetbox import OffsetImage, AnnotationBbox

    ax.annotate(r'Start', xy=(x[0], y[0]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points', fontsize=16,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[0], y[0], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)

    ax.annotate(r'End', xy=(x[x.shape[0]-1], y[y.shape[0]-1]), xycoords='data',
                            xytext=(-5, 5), textcoords='offset points',
                            fontsize=16,
                            horizontalalignment='left',
                            verticalalignment='bottom',
                            zorder=101
                            )
    ax.scatter(x[x.shape[0]-1], y[y.shape[0]-1], marker='o', facecolor='k', s=40, alpha=1.0, zorder=103)


    plt.xlabel(r'X [$m$]', fontsize=15, fontweight='bold')
    plt.ylabel(r'Y [$m$]', fontsize=15, fontweight='bold')
    ax.legend(loc=1, prop={'size':15})
    plt.grid(True)
    fig.savefig("gt_maritime_hall_20180720-1644.png", dpi=fig.dpi)
    plt.show(block=True)

##########################################################################
position = np.column_stack((reference_position.x.values,
    reference_position.y.values,  reference_position.z.values ))

##########################################################################
maritime_hall_figure(1, reference_position, position, reference_position)

