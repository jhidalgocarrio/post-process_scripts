# Copyright (c) 2015, Zhenwen Dai
# Licensed under the BSD 3-clause license (see LICENSE.txt)

from __future__ import print_function
from evaluation import RMSE
from methods import GP_RBF, SVIGP_RBF, SparseGP_RBF, GP_MAT32, SparseGP_MAT32, GP_MAT52, SparseGP_MAT52
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
          'methods':[GP_RBF, SparseGP_RBF, GP_MAT32, SparseGP_MAT32, GP_MAT52, SparseGP_MAT52],
          'tasks':[ExoTerOdometryResiduals],
          'train_sampling_time':['1s'],
          'test_sampling_time':['80ms','1s'],
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

            m = method()

            for train_t in range(len(config['train_sampling_time'])):
                train_time = config['train_sampling_time'][train_t]
                print(bcolors.BOLD + 'Train sampling time: '+ bcolors.ENDC + bcolors.WARNING + train_time + bcolors.ENDC)

                train = dataset.get_training_data(train_time)
                t_st = time.time()
                m.fit(train)
                t_pd = time.time() - t_st

                pred_train_mean = m.predict(train[0])

                for test_t in range(len(config['test_sampling_time'])):
                    test_time = config['test_sampling_time'][test_t]
                    print(bcolors.BOLD + 'Test sampling time: '+ bcolors.ENDC + bcolors.WARNING + test_time + bcolors.ENDC)

                    test = dataset.get_test_data(test_time)
                    pred_test_mean = m.predict(test[0])

                    for ei in range(len(config['evaluations'])):
                        evalu = config['evaluations'][ei]()
                        eval_test = evalu.evaluate(test[1], pred_test_mean)
                        eval_train = evalu.evaluate(train[1], pred_train_mean)

                        print(bcolors.OKBLUE + 'With evaluation method '+evalu.name + bcolors.ENDC, end=' ')
                        print('ERROR Train ['+ bcolors.FAIL + str(eval_train.mean()) + bcolors.ENDC + ']', end=' ')
                        print('ERROR Test ['+ bcolors.FAIL + str(eval_test.mean()) + bcolors.ENDC + ']')

                        results[task_i, method_i, ei, train_t, test_t] = eval_test

                    results[task_i, method_i, -1, train_t, test_t] = t_pd
                    figure = ExoTerFigures()
                    figure.output(fig_num, dataset, m, pred_test_mean, 0, train_time, test_time)
                    fig_num = fig_num + 1

                    print('',end='')
                print()

    [out.output(config, results) for out in config['outputs']]
