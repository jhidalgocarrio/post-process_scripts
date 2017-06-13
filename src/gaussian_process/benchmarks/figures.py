#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2016-08-27 19:49:55

import abc
import os
import numpy as np
from pylab import *
from matplotlib import pyplot as plt
from scipy.signal import butter, lfilter, freqz
import pandas as pandas
#pandas.set_option('display.mpl_style', 'default') # Make the graphs a bit prettier

class Figures(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def output(self, config, results):
        """Return the figure"""
        return None

    def butter_lowpass(self, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def butter_lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y

class ExoTerFigures(Figures):

    def output_velocity(self, fig_num, dataset, method,
            train_sampling_time, test_sampling_time, cutoff=0.1):

        #####################
        ### PLOT RESULTS  ###
        ######################
        matplotlib.style.use('classic') #in matplotlib >= 1.5.1
        matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
        fig, ax = plt.subplots()

        ref_velocity = dataset.test_reference_velocity.resample(test_sampling_time).mean()[dataset.ref_testing_mask]
        ref_time = dataset.test_reference_velocity.resample(test_sampling_time).mean().index.to_datetime()[dataset.ref_testing_mask]
        ax.plot(ref_time, ref_velocity.x,
                marker='o', linestyle='-', lw=4, alpha=0.5,
                color=[1.0, 0, 0],
                label='reference velocity')

        #ax.scatter(dataset.test_reference_velocity.index.to_datetime(), dataset.test_reference_velocity.x,
        #        marker='D', color=[1.0,0.0,0.0], s=20, alpha=0.2)

        ax.plot(dataset.test_odometry_velocity.index.to_datetime(), dataset.test_odometry_velocity.x,
                marker='D', linestyle='-', lw=4, alpha=0.5,
                color=[0, 0, 1.0],
                label='odometry velocity')

        formatted_ticks = dataset.test_reference_velocity.resample('5min').mean().index.map(lambda t: t.strftime('%H:%M:%S'))
        plt.xticks(dataset.test_reference_velocity.resample('5min').mean().index.to_pydatetime(), formatted_ticks)

        plt.xlabel(r'Time', fontsize=35, fontweight='bold')
        plt.ylabel(r'Velocity [$m/s$]', fontsize=35, fontweight='bold')
        plt.grid(True)
        ax.set_ylim([-0.10, 0.10])
        ax.legend(loc=4, prop={'size':40})
        title_str = dataset.name+"_" + method.name + "_train_at_"+train_sampling_time+"_test_at_"+test_sampling_time+"_velocity"
        #plt.title(title_str)
        plt.show(block=True)
        fig.savefig(title_str+".pdf", dpi=fig.dpi)
        return None

    def output_error(self, fig_num, dataset, method, prediction_mean, prediction_var,
            train_sampling_time, test_sampling_time, cutoff=0.1):

        #######################################
        # TEST DATA
        #######################################
        fig_time = dataset.test_odometry_velocity.resample(test_sampling_time).mean().index.to_datetime()[dataset.odo_testing_mask]
        #######################################

        ###################
        ### IIR FILTER  ###
        ###################
        order = 8
        fs = 1.0/float(test_sampling_time[0]) # sample rate, Hz

        matplotlib.style.use('classic') #in matplotlib >= 1.5.1

        #####################
        ### PLOT RESULTS  ###
        ######################
        matplotlib.style.use('classic') #in matplotlib >= 1.5.1
        matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
        fig, ax = plt.subplots()

        #filter the data
        #
        residual_test = self.butter_lowpass_filter(dataset.test[1][:,0], cutoff, fs, order)
        ax.plot(fig_time, residual_test,
                marker='x', linestyle='-', lw=4, alpha=1.0,
                color=[0, 0, 0],
                label='odometry error')

        ax.plot(fig_time, prediction_mean[:,0],
                marker='x', linestyle='-', lw=4, alpha=1.0,
                color=[1.0, 0.0, 0],
                label='GP prediction')

        #sigma = prediction_mean[:,0] + 3.0 * prediction_var[:,0]
        #ax.plot(dataset.test_odometry_velocity.index.to_datetime(), sigma,
        #        marker='', linestyle='--', lw=2,
        #        color=[1.0, 0.7, 0.0])

        #sigma = prediction_mean[:,0] - 3.0 * prediction_var[:,0]
        #ax.plot(dataset.test_odometry_velocity.index.to_datetime(), sigma,
        #        marker='', linestyle='--', lw=2,
        #        color=[1.0, 0.7, 0.0])

        formatted_ticks = dataset.test_odometry_velocity.resample('5min').mean().index.map(lambda t: t.strftime('%H:%M:%S'))
        plt.xticks(dataset.test_odometry_velocity.resample('5min').mean().index.to_pydatetime(), formatted_ticks)

        plt.xlabel(r'Time', fontsize=35, fontweight='bold')
        plt.ylabel(r'Velocity [$m/s$]', fontsize=35, fontweight='bold')
        plt.grid(True)
        ax.legend(loc=4, prop={'size':40})
        title_str = dataset.name+"_" + method.name + "_train_at_"+train_sampling_time+"_test_at_"+test_sampling_time+"_error"
        #plt.title(title_str)
        plt.show(block=True)
        fig.savefig(title_str+".pdf", dpi=fig.dpi)
        return None


    def output(self, fig_num, dataset, method, prediction_mean, prediction_var,
            train_sampling_time, test_sampling_time, cutoff=0.1):

        #######################################
        # TEST DATA
        #######################################
        fig_time = dataset.test_odometry_velocity.resample(test_sampling_time).mean().index.to_datetime()[dataset.odo_testing_mask]
        #######################################

        ###################
        ### IIR FILTER  ###
        ###################
        order = 8
        fs = 1.0/float(test_sampling_time[0]) # sample rate, Hz

        matplotlib.style.use('classic') #in matplotlib >= 1.5.1
        matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
        fig = plt.figure(fig_num, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
        ax = fig.add_subplot(111)

        ref_velocity = dataset.test_reference_velocity.resample(test_sampling_time).mean()[dataset.ref_testing_mask]
        ref_time = dataset.test_reference_velocity.resample(test_sampling_time).mean().index.to_datetime()[dataset.ref_testing_mask]
        ax.plot(ref_time, ref_velocity.x,
                marker='o', linestyle='-', lw=4, alpha=0.3,
                color=[1.0, 0, 0],
                label='reference velocity')

        #ax.scatter(dataset.test_reference_velocity.index.to_datetime(), dataset.test_reference_velocity.x,
        #        marker='D', color=[1.0,0.0,0.0], s=20, alpha=0.2)

        ax.plot(dataset.test_odometry_velocity.index.to_datetime(), dataset.test_odometry_velocity.x,
                marker='D', linestyle='-', lw=4, alpha=0.2,
                color=[0, 0, 1.0],
                label='odometry velocity')

        ax.set_ylim([-0.10, 0.10])
        ax.legend(loc=4, prop={'size':30})

        #filter the data
        #
        ax2 = fig.add_subplot(211)
        residual_test = self.butter_lowpass_filter(dataset.test[1][:,0], cutoff, fs, order)
        ax2.plot(fig_time, residual_test,
                marker='x', linestyle='-', lw=4, alpha=1.0,
                color=[0, 0, 0],
                label='odometry error')

        ax2.plot(fig_time, prediction_mean[:,0],
                marker='x', linestyle='-', lw=4, alpha=1.0,
                color=[1.0, 1.0, 0],
                label='GP prediction')

        #sigma = prediction_mean[:,0] + 3.0 * prediction_var[:,0]
        #ax.plot(dataset.test_odometry_velocity.index.to_datetime(), sigma,
        #        marker='', linestyle='--', lw=2,
        #        color=[1.0, 0.7, 0.0])

        #sigma = prediction_mean[:,0] - 3.0 * prediction_var[:,0]
        #ax.plot(dataset.test_odometry_velocity.index.to_datetime(), sigma,
        #        marker='', linestyle='--', lw=2,
        #        color=[1.0, 0.7, 0.0])

        formatted_ticks = dataset.test_odometry_velocity.resample('5min').mean().index.map(lambda t: t.strftime('%H:%M:%S'))
        plt.xticks(dataset.test_odometry_velocity.resample('5min').mean().index.to_pydatetime(), formatted_ticks)

        plt.xlabel(r'Time', fontsize=35, fontweight='bold')
        plt.ylabel(r'Velocity [$m/s$]', fontsize=35, fontweight='bold')
        plt.grid(True)
        ax2.legend(loc=4, prop={'size':40})
        title_str = dataset.name+"_" + method.name + "_train_at_"+train_sampling_time+"_test_at_"+test_sampling_time
        #plt.title(title_str)
        #plt.show(block=False)
        fig.savefig(title_str+".pdf", dpi=fig.dpi)
        return None

