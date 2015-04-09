#!/usr/bin/env python

path = '/home/javi/flatfish/development/data/20150312_dagon_estimator_evaluation_1h/20150312-1715/plots/'

#######################################
path_odometry_file = path + 'pose_odo_position.csv'

path_reference_file = path + 'pose_ref_position.csv'
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


#Dagon Odometry
odometry = data.ThreeData()
odometry.readData(path_odometry_file, cov=False)
odometry.eigenValues()

#Reference Pose
reference = data.ThreeData()
reference.readData(path_reference_file, cov=False)
reference.eigenValues()

