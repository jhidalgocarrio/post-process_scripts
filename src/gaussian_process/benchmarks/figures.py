#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2016-08-27 19:49:55

import abc
import os
import numpy as np
from pylab import *
from matplotlib import pyplot as plt
import pandas as pandas
matplotlib.style.use('ggplot') #in matplotlib >= 1.5.1
#pandas.set_option('display.mpl_style', 'default') # Make the graphs a bit prettier

class Figures(object):
    __metaclass__ = abc.ABCMeta
    
    @abc.abstractmethod
    def output(self, config, results):
        """Return the figure"""
        return None

class ExoTerFigures(Figures):
    def output(self, fig_num, dataset, method, prediction_mean, prediction_var, train_sampling_time, test_sampling_time):

        #######################################
        # TEST DATA
        #######################################
        fig_time = dataset.test_odometry_velocity.resample(test_sampling_time).mean().index.to_datetime()[dataset.testing_mask]
        #######################################

        matplotlib.rcParams.update({'font.size': 15, 'font.weight': 'bold'})
        fig = plt.figure(fig_num, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
        ax = fig.add_subplot(111)

        ax.plot(dataset.test_reference_velocity.index.to_datetime(), dataset.test_reference_velocity.x,
                marker='o', linestyle='-', lw=1, alpha=0.3,
                color=[1.0, 0, 0],
                label='Reference velocity')

        ax.scatter(dataset.test_reference_velocity.index.to_datetime(), dataset.test_reference_velocity.x,
                marker='D', color=[1.0,0.0,0.0], s=20, alpha=0.2)

        ax.plot(dataset.test_odometry_velocity.index.to_datetime(), dataset.test_odometry_velocity.x,
                marker='x', linestyle='-', lw=1, alpha=0.2,
                color=[0, 0, 1.0],
                label='Odometry velocity')


        ax.plot(fig_time, dataset.test[1][:,0],
                marker='x', linestyle='-', lw=4, alpha=1.0,
                color=[0, 0, 0],
                label='Error')

        ax.plot(fig_time, prediction_mean[:,0],
                marker='x', linestyle='-', lw=4, alpha=1.0,
                color=[1.0, 1.0, 0],
                label='Prediction')

        #sigma = prediction_mean[:,0] + 3.0 * prediction_var[:,0]
        #ax.plot(dataset.test_odometry_velocity.index.to_datetime(), sigma,
        #        marker='', linestyle='--', lw=2,
        #        color=[1.0, 0.7, 0.0])

        #sigma = prediction_mean[:,0] - 3.0 * prediction_var[:,0]
        #ax.plot(dataset.test_odometry_velocity.index.to_datetime(), sigma,
        #        marker='', linestyle='--', lw=2,
        #        color=[1.0, 0.7, 0.0])

        plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
        plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
        plt.grid(True)
        ax.legend(loc=1, prop={'size':15})
        title_str = "ExoTerOdometryResiduals:_" + method.name + "_train_at_"+train_sampling_time+"_test_at_"+test_sampling_time
        plt.title(title_str)
        #plt.show(block=False)
        fig.savefig(title_str+".png", dpi=fig.dpi)
        return None

