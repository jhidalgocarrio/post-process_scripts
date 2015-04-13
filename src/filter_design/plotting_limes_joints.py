#!/usr/bin/env python

path = '/home/javi/esa-npi/development/data/20150410_spaceclimber_simple_test/incontact_ground/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'
#######################################


import sys
sys.path.insert(0, './src/core')
import numpy as np
from pylab import *
from matplotlib import pyplot as plt
import quaternion as quat
import datadisplay as data
import cov_ellipse as cov
import joints as js

# Robot Joints Position and Speed
names = ["JointBody", "leg0/joint0", "leg0/joint1", "leg0/joint2",
    "leg0/joint3", "leg0/joint4", "leg0/contact", "leg0/slipx", "leg0/slipy",
    "leg0/slipz", "leg1/joint0", "leg1/joint1", "leg1/joint2", "leg1/joint3",
    "leg1/joint4", "leg1/contact", "leg1/slipx", "leg1/slipy", "leg1/slipz",
    "leg2/joint0", "leg2/joint1", "leg2/joint2", "leg2/joint3", "leg2/joint4",
    "leg2/contact", "leg2/slipx", "leg2/slipy", "leg2/slipz", "leg3/joint0",
    "leg3/joint1", "leg3/joint2", "leg3/joint3", "leg3/joint4", "leg3/contact",
    "leg3/slipx", "leg3/slipy", "leg3/slipz", "leg4/joint0", "leg4/joint1",
    "leg4/joint2", "leg4/joint3", "leg4/joint4", "leg4/contact", "leg4/slipx",
    "leg4/slipy", "leg4/slipz", "leg5/joint0", "leg5/joint1", "leg5/joint2",
    "leg5/joint3", "leg5/joint4", "leg5/contact", "leg5/slipx", "leg5/slipy",
    "leg5/slipz"]

robot_joints = js.Joints(names)
robot_joints.readData(joints_position_file, joints_speed_file)

