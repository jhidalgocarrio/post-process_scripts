# Copyright (c) 2015, Zhenwen Dai
# Licensed under the BSD 3-clause license (see LICENSE.txt)

from __future__ import print_function
from evaluation import RMSE
from methods import GP_RBF, SVIGP_RBF, SparseGP_RBF, GP_MAT32, SparseGP_MAT32
from tasks import ExoTerOdometryResiduals
from outputs import ScreenOutput, CSVOutput, H5Output
from figures import ExoTerFigures
import numpy as np
import time

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


outpath = '.'
prjname = 'exoter_odometry_residual_regression'
config = {
          'evaluations':[RMSE],
          'methods':[GP_RBF, SparseGP_RBF, GP_MAT32, SparseGP_MAT32],
          'tasks':[ExoTerOdometryResiduals],
          'train_sampling_time':['500ms', '1s', '2s', '5s'],
          'test_sampling_time':['500ms', '1s', '2s', '5s'],
          'outputs': [ScreenOutput()]
          #'outputs': [ScreenOutput(), CSVOutput(outpath, prjname)]
          }

if __name__=='__main__':
    results = np.zeros((len(config['tasks']), len(config['methods']), len(config['evaluations'])+1, len(config['train_sampling_time']), len(config['test_sampling_time'])))

    fig_num = 0

    for task_i in range(len(config['tasks'])):
        dataset = config['tasks'][task_i]()
        print(bcolors.HEADER  + bcolors.BOLD + 'Benchmarking on '+dataset.name + bcolors.ENDC)
        res = dataset.load_data()
        if not res: print('Fail to load '+config['tasks'][task_i].name); continue

        for method_i in range(len(config['methods'])):
            method = config['methods'][method_i]
            print(bcolors.WARNING + 'With the method '+method.name + bcolors.ENDC)

            for train_t in range(len(config['train_sampling_time'])):
                train_time = config['train_sampling_time'][train_t]
                print(bcolors.BOLD + 'Train sampling time: '+ bcolors.ENDC + bcolors.WARNING + train_time + bcolors.ENDC)
                train = dataset.get_training_data(train_time)

                for test_t in range(len(config['test_sampling_time'])):
                    test_time = config['test_sampling_time'][test_t]
                    print(bcolors.BOLD + 'Test sampling time: '+ bcolors.ENDC + bcolors.WARNING + test_time + bcolors.ENDC)
                    test = dataset.get_test_data(test_time)

                    m = method()
                    t_st = time.time()
                    m.fit(train)
                    pred_mean = m.predict(test[0])
                    t_pd = time.time() - t_st

                    print (bcolors.OKGREEN + "Prediction shape" + str(pred_mean.shape) + bcolors.ENDC)

                    for ei in range(len(config['evaluations'])):
                        evalu = config['evaluations'][ei]()
                        print(bcolors.OKBLUE + 'With evaluation method '+evalu.name + bcolors.ENDC)
                        results[task_i, method_i, ei, train_t, test_t] = evalu.evaluate(test[1], pred_mean)
                    results[task_i, method_i, -1, train_t, test_t] = t_pd

                    figure = ExoTerFigures()
                    figure.output(fig_num, dataset, m, pred_mean, 0, train_time, test_time)
                    fig_num = fig_num + 1

                    print('',end='')
                print()

    [out.output(config, results) for out in config['outputs']]
