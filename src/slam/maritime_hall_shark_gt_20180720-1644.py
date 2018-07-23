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
import rosbag
from std_msgs.msg import Int32, String
import numpy as np


#import rosbag_pandas
#df = rosbag_pandas.bag_to_dataframe(os.path.expanduser(path_reference_position_file)) #awesome data processing


# Process the ROS bag logs
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


