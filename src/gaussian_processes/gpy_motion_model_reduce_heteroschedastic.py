#!/usr/bin/env python

path = '/home/javi/exoter/development/data/20141023_pink_test/20141023-2011/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

pose_ref_velocity_file =  path + 'pose_ref_velocity.0.data'

pose_odo_velocity_file =  path + 'pose_odo_velocity.0.data'

pose_imu_orientation_file =  path + 'pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data'
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
import GPy

# Reference Robot Velocity
reference_velocity = data.ThreeData()
reference_velocity.readData(pose_ref_velocity_file, cov=True)
reference_velocity.eigenValues()

# Odometry Robot Velocity
odometry_velocity = data.ThreeData()
odometry_velocity.readData(pose_odo_velocity_file, cov=True)
odometry_velocity.eigenValues()

# IMU orientation
imu_orient = data.QuaternionData()
imu_orient.readData(pose_imu_orientation_file, cov=True)
imu_orient.eigenValues()

# IMU acceleration
imu_acc = data.ThreeData()
imu_acc.readData(pose_imu_acceleration_file, cov=False)
imu_acc.eigenValues()

# IMU Angular Velocity
imu_gyro = data.ThreeData()
imu_gyro.readData(pose_imu_angular_velocity_file, cov=False)
imu_gyro.eigenValues()

# Robot Joints Position and Speed
names = "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"

robot_joints = js.Joints(names)
robot_joints.readData(joints_position_file, joints_speed_file)

#########################
## LOAD INPUT TEST    ##
#########################

np.random.seed(1)

#################
# Joints Inputs #
joints = np.column_stack((
                        robot_joints.getSpeed("fl_translation"),
                        robot_joints.getSpeed("fr_translation"),
                        robot_joints.getSpeed("ml_translation"),
                        robot_joints.getSpeed("mr_translation"),
                        robot_joints.getSpeed("rl_translation"),
                        robot_joints.getSpeed("rr_translation"),
                        robot_joints.getPosition("fl_steer"),
                        robot_joints.getPosition("fr_steer"),
                        robot_joints.getPosition("rl_steer"),
                        robot_joints.getPosition("rr_steer"),
                        robot_joints.getPosition("left_passive"),
                        robot_joints.getPosition("right_passive"),
                        robot_joints.getPosition("rear_passive")
                        ))
##################
# Inertia Inputs #
inertia = np.column_stack((imu_acc.getAxis(0), imu_acc.getAxis(1), imu_acc.getAxis(2),
                           imu_gyro.getAxis(0), imu_gyro.getAxis(1), imu_gyro.getAxis(2)))

######################
# Orientation Inputs #
# Roll and Pitch angles in that order #
orient = np.column_stack((imu_orient.getEuler(2), imu_orient.getEuler(1)))

###########################
# Create Reference Output #
reference = np.column_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2)))

#########################
## SPLIT INPUT TEST    ##
#########################
def input_reduction (input_array = None, number_blocks = 0.0):
    if input_array is None:
        raise ValueError("Input cannot be None")
    if number_blocks is 0.0:
        raise ValueError("Cannot split array with zero number of blocks")

    new_input = np.ndarray(shape = (number_block, input_array.shape[1]))
    std_input = np.ndarray(shape = (number_block, input_array.shape[1]))
    for i in range(input_array.shape[1]):
        split_array = np.array_split(input_array[:,i], block_size)

        mean_array = np.ndarray(len(split_array), dtype=double)
        std_array = np.ndarray(len(split_array), dtype=double)

        for j in range(len(split_array)):
            mean_array[j] = mean(split_array[j])
            std_array[j] = std(split_array[j])

        new_input[:,i] = mean_array
        std_input[:,i] = std_array

    return new_input, std_input
#########################

sampling_frequency = 1.0/mean(reference_velocity.delta)
size_block = 1 * sampling_frequency
number_blocks = int(len(reference_velocity.delta)/size_block)

# Split joints (one joint info per column)
joints, jointstd = input_reduction(joints, size_block)

# Split inertia (one axis info per column)
inertia, inertiastd = input_reduction(inertia, size_block)

# Split orientation (one axis info per column)
orient, orientstd = input_reduction(orient, size_block)

# Split orientation (one axis info per column)
reference, referencestd = input_reduction(reference, size_block)
