#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2018-07-23 14:10:35
#######################################
path='~/waste_shark/experiments/20181115_maritime_hall/'
#######################################
path_reference_position_file = path + '2018-11-15-18-37-26.bag'
#path_slam_position_file = path + '2018-11-19-18-37-16.bag'
path_slam_position_file = path + '2018-11-22-18-28-40.bag'
#######################################

import sys
import os
sys.path.insert(0, './src/core')
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
# to get the ground truth
############################
bag = rosbag.Bag(os.path.expanduser(path_reference_position_file))
pose_time = []
pose_pos = []
pose_orient = []
for topic, msg, t in bag.read_messages(topics=['/dory/odom']):
        print "-----"
        print topic
        print "-----"
        print msg
        print "-----"
        print t
        pose_time.append(t.to_time())
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose_pos.append(np.array([position.x, position.y, position.z]))
        pose_orient.append(quat.quaternion([orientation.w, orientation.x, orientation.y, orientation.z]))

pose_time = np.asarray(pose_time)
pose_pos = np.asarray(pose_pos)

april_tag_pos=[]
april_tag_orient=[]
for topic, msg, t in bag.read_messages(topics=['/tf_static']):
    if msg.transforms[0].child_frame_id == 'april_tag_link':
        print "-----"
        print msg
        print "-----"
        print t
        april_tag_pos = np.array([msg.transforms[0].transform.translation.x,
                msg.transforms[0].transform.translation.y,
                msg.transforms[0].transform.translation.z])
        april_tag_orient = quat.quaternion([msg.transforms[0].transform.rotation.w,
                msg.transforms[0].transform.rotation.x,
                msg.transforms[0].transform.rotation.y,
                msg.transforms[0].transform.rotation.z])

pose_orient[:] = [x * april_tag_orient.__invert__() for x in pose_orient]
pose_pos[:] = [x - april_tag_pos for x in pose_pos]

############################
# Convert to pandas
############################
reference_position = pandas.DataFrame(data=pose_pos, index=[datetime.datetime.fromtimestamp(x) for x in pose_time], columns=['x', 'y', 'z'])
reference_orient = pandas.DataFrame(data=np.asarray(pose_orient), index=[datetime.datetime.fromtimestamp(x) for x in pose_time], columns=['w', 'x', 'y', 'z'])

############################
# Process the ROS bag logs
# to get the SLAM pose
############################
bag = rosbag.Bag(os.path.expanduser(path_slam_position_file))
pose_time = []
pose_pos = []
pose_orient = []
for topic, msg, t in bag.read_messages(topics=['/dory/dory_slam/pose']):
        pose_time.append(t.to_time())
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose_pos.append(np.array([position.x, position.y, position.z]))
        pose_orient.append(quat.quaternion([orientation.w, orientation.x, orientation.y, orientation.z]))

############################
# Convert to pandas
############################
slam_position = pandas.DataFrame(data=pose_pos, index=[datetime.datetime.fromtimestamp(x) for x in pose_time], columns=['x', 'y', 'z'])
slam_orient = pandas.DataFrame(data=np.asarray(pose_orient), index=[datetime.datetime.fromtimestamp(x) for x in pose_time], columns=['w', 'x', 'y', 'z'])

##########################################################################
# PLOTTING FUNCTION
##########################################################################
def maritime_hall_figure(fig_num, ground_file, reference_trajectory,
        slam_trajectory,
        mis_orient = quat.quaternion(np.array([1.0,0.0,0.0,0.0])),
        mis_position = [0.00, 0.00, 0.00]):

    import os
    from matplotlib.cbook import get_sample_data
    from matplotlib._png import read_png
    import matplotlib.image as image
    matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
    #fig = plt.figure(fig_num, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
    #ax = fig.add_subplot(111)
    fn = get_sample_data(os.getcwd()+ground_file, asfileobj=False)
    maritime_hall = image.imread(fn)
    fig, ax = plt.subplots()
    ax.imshow(maritime_hall, extent=[-1.2, 25, -2, 20])
    #ax.imshow(maritime_hall, extent=[-1.2, 25, -2, 19])


    # Display Ground Truth trajectory
    from numpy import linalg as la
    ref = np.column_stack((reference_trajectory.x.values[0::1],
                            reference_trajectory.y.values[0::1],
                            reference_trajectory.z.values[0::1]))
    ref[:] = [(mis_orient * i * mis_orient.conj())[1:4] for i in ref ]
    ref[:] = [ i + mis_position for i in ref ]
    x = ref[:,0]
    y = ref[:,1]
    ax.plot(x, y, marker='D', linestyle='-', lw=2, alpha=0.5, color=[1.0, 1.0, 0.0],
            label='ground truth', zorder=80)

    # SLAM trajectory
    slam = np.column_stack((slam_trajectory.x.values[0::1],
                            slam_trajectory.y.values[0::1],
                            slam_trajectory.z.values[0::1]))
    slam[:] = [(mis_orient * i * mis_orient.conj())[1:4] for i in slam ]
    slam[:] = [ i + mis_position for i in slam ]
    x = slam[:,0]
    y = slam[:,1]
    ax.plot(x, y, linestyle='-', marker='s', lw=2, alpha=0.5, color=[0.0, 1.0, 0.3],
            label='slam trajectory',
            zorder=98)
    ax.scatter(x, y, marker='s', facecolor=[0.0,1.0,0.3], edgecolor='g', alpha=0.5, zorder=99)

    # Annotations
    from scipy import ndimage
    from matplotlib.offsetbox import OffsetImage, AnnotationBbox
    fn = get_sample_data(os.getcwd()+"/data/img/wasteshark.png", asfileobj=False)
    waste_shark = image.imread(fn)
    waste_shark = ndimage.rotate(waste_shark, 90)
    imws = OffsetImage(waste_shark, zoom=0.1)

    ab = AnnotationBbox(imws, xy=(x[0], y[0]),
                    xybox=None,
                    xycoords='data',
                    boxcoords="offset points",
                    frameon=False)


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

    ax.add_artist(ab)

    plt.xlabel(r'X [$m$]', fontsize=15, fontweight='bold')
    plt.ylabel(r'Y [$m$]', fontsize=15, fontweight='bold')
    ax.legend(loc=1, prop={'size':15})
    plt.grid(True)
    fig.savefig("gt_maritime_hall_20180720-1644.png", dpi=fig.dpi)
    plt.show(block=True)

##########################################################################

##########################################################################
maritime_hall_figure(1, "/data/img/maritime_hall.png", reference_position, slam_position)

