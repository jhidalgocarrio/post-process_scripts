# Copyright (c) 2015, Zhenwen Dai
# Licensed under the BSD 3-clause license (see LICENSE.txt)

import sys
sys.path.insert(0, './src/core')
from pylab import *
import quaternion as quat
import datadisplay as data
import abc
import os
import numpy as np
import pandas as pandas
import datetime

def dateparse (time_in_microsecs):
        return datetime.datetime.fromtimestamp(float(time_in_microsecs * 1e-06))

class RegressionTask(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, datapath='./'):
        self.datapath = datapath

    @abc.abstractmethod
    def load_data(self):
        """Download the dataset if not exist. Return True if successful"""
        return True

    @abc.abstractmethod
    def get_training_data(self, sampling_time):
        """Return the training data: training data and labels"""
        return None

    @abc.abstractmethod
    def get_test_data(self, sampling_time):
        """Return the test data: training data and labels"""
        return None



class ExoTerOdometryResiduals(RegressionTask):

    name='ExoTerOdometryResiduals'

    #######################################
    # TRAINING DATA
    #######################################
    url_train = '/home/javi/exoter/development/data/20140000_gaussian_processes/merged_bis/'
    #######################################
    train_joints_position_file = url_train + 'joints_position.0.data'

    train_joints_speed_file = url_train + 'joints_speed.0.data'

    train_pose_ref_velocity_file =  url_train + 'delta_pose_ref_velocity.0.data'

    train_pose_odo_velocity_file =  url_train + 'delta_pose_odo_velocity.0.data'

    train_pose_odo_orientation_file =  url_train + 'pose_odo_orientation.0.data'

    train_pose_imu_orientation_file =  url_train + 'pose_imu_orientation.0.data'

    train_pose_imu_angular_velocity_file =  url_train + 'pose_imu_angular_velocity.0.data'

    train_pose_imu_acceleration_file =  url_train + 'pose_imu_acceleration.0.data'

    #######################################
    # TEST DATA
    #######################################
    url_test = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/'
    #######################################
    test_joints_position_file = url_test + 'joints_position.0.data'

    test_joints_speed_file = url_test + 'joints_speed.0.data'

    test_pose_ref_velocity_file =  url_test + 'delta_pose_ref_velocity.0.data'

    test_pose_odo_velocity_file =  url_test + 'delta_pose_odo_velocity.0.data'

    test_pose_odo_orientation_file =  url_test + 'pose_odo_orientation.0.data'

    test_pose_imu_orientation_file =  url_test + 'pose_imu_orientation.0.data'

    test_pose_imu_angular_velocity_file =  url_test + 'pose_imu_angular_velocity.0.data'

    test_pose_imu_acceleration_file =  url_test + 'pose_imu_acceleration.0.data'


    def load_data(self):

        ##########################################################################
        # TRAINING
        ##########################################################################

        ##########################################################################
        # READ THE VALUES IN PANDAS
        ##########################################################################

        # Reference Robot Velocity
        self.train_reference_velocity = pandas.read_csv(self.train_pose_ref_velocity_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time',
            names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
                'cov_zx', 'cov_zy', 'cov_zz'], header=None)

        # Odometry Robot Velocity
        self.train_odometry_velocity = pandas.read_csv(self.train_pose_odo_velocity_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time',
            names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
                'cov_zx', 'cov_zy', 'cov_zz'], header=None)

        # IMU orientation (in quaternion form)
        train_data_orient = data.QuaternionData()
        train_data_orient.readData(self.train_pose_imu_orientation_file, cov=True)
        time = [dateparse(x*1e06) for x in train_data_orient.atime]
        euler = np.column_stack((train_data_orient.getEuler(2), train_data_orient.getEuler(1), train_data_orient.getEuler(0)))
        self.train_imu_orient = pandas.DataFrame(data=euler, columns=['x', 'y', 'z'], index=time)

        # IMU acceleration
        self.train_imu_acc = pandas.read_csv(self.train_pose_imu_acceleration_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time',
            names=['time', 'x', 'y', 'z'], header=None)

        # IMU Angular Velocity
        self.train_imu_gyro = pandas.read_csv(self.train_pose_imu_angular_velocity_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time',
            names=['time', 'x', 'y', 'z'], header=None)

        # Robot Joints Position and Speed
        names = ["time", "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"]

        self.train_joints_position = pandas.read_csv(self.train_joints_position_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time', names=names, header=None)

        self.train_joints_speed = pandas.read_csv(self.train_joints_speed_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time', names=names, header=None)

        ##########################################################################
        # REMOVE OUTLIERS
        ##########################################################################
        # ExoTer Max velocity is 10cm/s
        self.train_reference_velocity = self.train_reference_velocity.drop(self.train_reference_velocity[fabs(self.train_reference_velocity.x) > 0.15].index)
        self.train_odometry_velocity = self.train_odometry_velocity.drop(self.train_odometry_velocity[fabs(self.train_odometry_velocity.x) > 0.15].index)

        ##########################################################################
        # TEST
        ##########################################################################

        ##########################################################################
        # READ THE VALUES IN PANDAS
        ##########################################################################

        # Reference Robot Velocity
        self.test_reference_velocity = pandas.read_csv(self.test_pose_ref_velocity_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time',
            names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
                'cov_zx', 'cov_zy', 'cov_zz'], header=None)

        # Odometry Robot Velocity
        self.test_odometry_velocity = pandas.read_csv(self.test_pose_odo_velocity_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time',
            names=['time', 'x', 'y', 'z', 'cov_xx', 'cov_xy', 'cov_xz', 'cov_yx', 'cov_yy', 'cov_yz',
                'cov_zx', 'cov_zy', 'cov_zz'], header=None)

        # IMU orientation (in quaternion form)
        test_data_orient = data.QuaternionData()
        test_data_orient.readData(self.test_pose_imu_orientation_file, cov=True)
        time = [dateparse(x*1e06) for x in test_data_orient.atime]
        euler = np.column_stack((test_data_orient.getEuler(2), test_data_orient.getEuler(1), test_data_orient.getEuler(0)))
        self.test_imu_orient = pandas.DataFrame(data=euler, columns=['x', 'y', 'z'], index=time)

                # IMU acceleration
        self.test_imu_acc = pandas.read_csv(self.test_pose_imu_acceleration_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time',
            names=['time', 'x', 'y', 'z'], header=None)

                # IMU Angular Velocity
        self.test_imu_gyro = pandas.read_csv(self.test_pose_imu_angular_velocity_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time',
            names=['time', 'x', 'y', 'z'], header=None)

                # Robot Joints Position and Speed
        names = ["time", "left_passive", "fl_mimic", "fl_walking", "fl_steer", "fl_drive", "fl_contact", "fl_translation", "fl_slipx", "fl_slipy", "fl_slipz", "ml_mimic", "ml_walking", "ml_drive", "ml_contact", "ml_translation", "ml_slipx", "ml_slipy", "ml_slipz", "rear_passive", "rl_mimic", "rl_walking", "rl_steer", "rl_drive", "rl_contact", "rl_translation", "rl_slipx", "rl_slipy", "rl_slipz", "rr_mimic", "rr_walking", "rr_steer", "rr_drive", "rr_contact", "rr_translation", "rr_slipx", "rr_slipy", "rr_slipz", "right_passive", "fr_mimic", "fr_walking", "fr_steer", "fr_drive", "fr_contact", "fr_translation", "fr_slipx", "fr_slipy", "fr_slipz", "mr_mimic", "mr_walking", "mr_drive", "mr_contact", "mr_translation", "mr_slipx", "mr_slipy", "mr_slipz"]

        self.test_joints_position = pandas.read_csv(self.test_joints_position_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time', names=names, header=None)

        self.test_joints_speed = pandas.read_csv(self.test_joints_speed_file, sep=" ", parse_dates=True,
            date_parser=dateparse , index_col='time', names=names, header=None)

        ##########################################################################
        # REMOVE OUTLIERS
        ##########################################################################
        # ExoTer Max velocity is 10cm/s
        self.test_reference_velocity = self.test_reference_velocity.drop(self.test_reference_velocity[fabs(self.test_reference_velocity.x) > 0.15].index)
        self.test_odometry_velocity = self.test_odometry_velocity.drop(self.test_odometry_velocity[fabs(self.test_odometry_velocity.x) > 0.15].index)

        return True

    def get_training_data(self, resampling_time):

        #################
        ## RE-SAMPLE
        #################
        reference_velocity = self.train_reference_velocity.resample(resampling_time)
        odometry_velocity = self.train_odometry_velocity.resample(resampling_time)
        imu_orient = self.train_imu_orient.resample(resampling_time)
        imu_acc = self.train_imu_acc.resample(resampling_time)
        imu_gyro = self.train_imu_gyro.resample(resampling_time)
        joints_position = self.train_joints_position.resample(resampling_time)
        joints_speed = self.train_joints_speed.resample(resampling_time)

        #Compute the error in odometry
        odometry_velocity['error_x'] = pandas.Series (fabs(odometry_velocity.x - reference_velocity.x))
        odometry_velocity['error_y'] = pandas.Series (fabs(odometry_velocity.y - reference_velocity.y))
        odometry_velocity['error_z'] = pandas.Series (fabs(odometry_velocity.z - reference_velocity.z))

        ##########################################################################
        # ELIMINATE NULL VALUES
        ##########################################################################
        self.training_mask = pandas.notnull(odometry_velocity.error_x) & pandas.notnull(odometry_velocity.error_y) & pandas.notnull(odometry_velocity.error_z)

        reference_velocity = reference_velocity[self.training_mask]
        odometry_velocity = odometry_velocity[self.training_mask]
        imu_orient = imu_orient[self.training_mask]
        imu_acc = imu_acc[self.training_mask]
        imu_gyro = imu_gyro[self.training_mask]
        joints_position = joints_position[self.training_mask]
        joints_speed = joints_speed[self.training_mask]

        ##########################################################################
        # GAUSSIAN PROCESS X INPUT VECTOR
        ##########################################################################
        X_orientation = np.column_stack((
                imu_orient.x.as_matrix(),
                imu_orient.y.as_matrix()))

        X_joints_position = np.column_stack((
                joints_position.left_passive.as_matrix(),
                joints_position.right_passive.as_matrix(),
                joints_position.rear_passive.as_matrix(),
                joints_position.fl_steer.as_matrix(),
                joints_position.fr_steer.as_matrix(),
                joints_position.rl_steer.as_matrix(),
                joints_position.rr_steer.as_matrix(),
                ))

        X_joints_speed = np.column_stack((
                joints_speed.left_passive.as_matrix(),
                joints_speed.right_passive.as_matrix(),
                joints_speed.rear_passive.as_matrix(),
                joints_speed.fl_steer.as_matrix(),
                joints_speed.fr_steer.as_matrix(),
                joints_speed.rl_steer.as_matrix(),
                joints_speed.rr_steer.as_matrix(),
                joints_speed.fl_translation.as_matrix(),
                joints_speed.fr_translation.as_matrix(),
                joints_speed.ml_translation.as_matrix(),
                joints_speed.mr_translation.as_matrix(),
                joints_speed.rl_translation.as_matrix(),
                joints_speed.rr_translation.as_matrix(),
                ))

        X_acc =  np.column_stack((
                imu_acc.x.as_matrix(),
                imu_acc.y.as_matrix(),
                imu_acc.z.as_matrix()
                ))

        X_gyro =  np.column_stack((
                imu_gyro.x.as_matrix(),
                imu_gyro.y.as_matrix(),
                imu_gyro.z.as_matrix()
                ))

        X = np.column_stack((X_orientation, X_joints_position, X_joints_speed, X_acc,
            X_gyro))

        ##########################################################################
        # GAUSSIAN PROCESS Y OUTPUT VECTOR
        ##########################################################################
        Y = np.column_stack((odometry_velocity.error_x.as_matrix(),
                odometry_velocity.error_y.as_matrix(),
                odometry_velocity.error_z.as_matrix()))

        self.train = (X, Y)

        print ("Compute training X"+str(self.train[0].shape)+" and Y"+str(self.train[1].shape)+" data at sampling time "+resampling_time)

        return self.train

    def get_test_data(self, resampling_time):

        #################
        ## RE-SAMPLE
        #################
        reference_velocity = self.test_reference_velocity.resample(resampling_time)
        odometry_velocity = self.test_odometry_velocity.resample(resampling_time)
        imu_orient = self.test_imu_orient.resample(resampling_time)
        imu_acc = self.test_imu_acc.resample(resampling_time)
        imu_gyro = self.test_imu_gyro.resample(resampling_time)
        joints_position = self.test_joints_position.resample(resampling_time)
        joints_speed = self.test_joints_speed.resample(resampling_time)

        #Compute the error
        odometry_velocity['error_x'] = pandas.Series (fabs(odometry_velocity.x - reference_velocity.x))
        odometry_velocity['error_y'] = pandas.Series (fabs(odometry_velocity.y - reference_velocity.y))
        odometry_velocity['error_z'] = pandas.Series (fabs(odometry_velocity.z - reference_velocity.z))
        ##########################################################################
        # ELIMINATE NULL VALUES
        ##########################################################################
        self.testing_mask = pandas.notnull(odometry_velocity.error_x) & pandas.notnull(odometry_velocity.error_y) & pandas.notnull(odometry_velocity.error_z)

        # Equalize the length of data
        reference_velocity = reference_velocity[0:self.testing_mask.shape[0]]
        odometry_velocity = odometry_velocity[0:self.testing_mask.shape[0]]
        imu_orient = imu_orient[0:self.testing_mask.shape[0]]
        imu_acc = imu_acc[0:self.testing_mask.shape[0]]
        imu_gyro = imu_gyro[0:self.testing_mask.shape[0]]
        joints_position = joints_position[0:self.testing_mask.shape[0]]
        joints_speed = joints_speed[0:self.testing_mask.shape[0]]

        # Sync index with odometry
        reference_velocity.index = self.testing_mask.index
        odometry_velocity.index = self.testing_mask.index
        imu_orient.index = self.testing_mask.index
        imu_acc.index = self.testing_mask.index
        imu_gyro.index = self.testing_mask.index
        joints_position.index = self.testing_mask.index
        joints_speed.index = self.testing_mask.index

        # Apply the mask
        reference_velocity = reference_velocity[self.testing_mask]
        odometry_velocity = odometry_velocity[self.testing_mask]
        imu_orient = imu_orient[self.testing_mask]
        imu_acc = imu_acc[self.testing_mask]
        imu_gyro = imu_gyro[self.testing_mask]
        joints_position = joints_position[self.testing_mask]
        joints_speed = joints_speed[self.testing_mask]

        ##########################################################################
        # GAUSSIAN PROCESS X INPUT VECTOR
        ##########################################################################
        Xp_orientation = np.column_stack((
                imu_orient.x.as_matrix(),
                imu_orient.y.as_matrix()))

        Xp_joints_position = np.column_stack((
                joints_position.left_passive.as_matrix(),
                joints_position.right_passive.as_matrix(),
                joints_position.rear_passive.as_matrix(),
                joints_position.fl_steer.as_matrix(),
                joints_position.fr_steer.as_matrix(),
                joints_position.rl_steer.as_matrix(),
                joints_position.rr_steer.as_matrix(),
                ))

        Xp_joints_speed = np.column_stack((
                joints_speed.left_passive.as_matrix(),
                joints_speed.right_passive.as_matrix(),
                joints_speed.rear_passive.as_matrix(),
                joints_speed.fl_steer.as_matrix(),
                joints_speed.fr_steer.as_matrix(),
                joints_speed.rl_steer.as_matrix(),
                joints_speed.rr_steer.as_matrix(),
                joints_speed.fl_translation.as_matrix(),
                joints_speed.fr_translation.as_matrix(),
                joints_speed.ml_translation.as_matrix(),
                joints_speed.mr_translation.as_matrix(),
                joints_speed.rl_translation.as_matrix(),
                joints_speed.rr_translation.as_matrix(),
                ))

        Xp_acc =  np.column_stack((
                imu_acc.x.as_matrix(),
                imu_acc.y.as_matrix(),
                imu_acc.z.as_matrix()
                ))

        Xp_gyro =  np.column_stack((
                imu_gyro.x.as_matrix(),
                imu_gyro.y.as_matrix(),
                imu_gyro.z.as_matrix()
                ))

        Xp = np.column_stack((Xp_orientation, Xp_joints_position, Xp_joints_speed, Xp_acc,
            Xp_gyro))

        ##########################################################################
        # GAUSSIAN PROCESS Y OUTPUT VECTOR
        ##########################################################################
        Yp = np.column_stack((odometry_velocity.error_x.as_matrix(),
                odometry_velocity.error_y.as_matrix(),
                odometry_velocity.error_z.as_matrix()))

        self.test = (Xp, Yp)

        print ("Compute test Xp"+str(self.test[0].shape)+" and Yp"+str(self.test[1].shape)+" data at sampling time "+resampling_time)

        return self.test

