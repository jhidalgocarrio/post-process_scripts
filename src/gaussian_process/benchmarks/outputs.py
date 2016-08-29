# Copyright (c) 2015, Zhenwen Dai
# Licensed under the BSD 3-clause license (see LICENSE.txt)

from __future__ import print_function
import abc
import os
import numpy as np
import datetime

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class Output(object):
    __metaclass__ = abc.ABCMeta
    
    @abc.abstractmethod
    def output(self, config, results):
        """Return the test data: training data and labels"""
        return None

class ScreenOutput(Output):

    def output(self, config, results):
        now = datetime.datetime.now()
        print(bcolors.BOLD + '='*20+' REPORT ['+now.strftime("%d.%m.%Y %H:%M")+'] '+'='*20 + bcolors.ENDC)

        for task_i in range(len(config['tasks'])):
            print(bcolors.WARNING + config['tasks'][task_i].name + bcolors.ENDC)

            for method_i in range(len(config['methods'])):
                for ei in range(len(config['evaluations'])):
                    print(bcolors.UNDERLINE + config['methods'][method_i].name+'('+config['evaluations'][ei].name + ')'+bcolors.ENDC, end='')
                    print(bcolors.UNDERLINE + '\t'.join([' ']+['train_sampling: '+train_t for train_t in config['train_sampling_time']]) + bcolors.ENDC)
                    for test_ti in range(len(config['test_sampling_time'])):
                        outputs = []
                        for train_ti in range(len(config['train_sampling_time'])):
                            m = results[task_i, method_i, ei, train_ti, test_ti].mean()
                            t = results[task_i, method_i, -1, train_ti, test_ti]
                            outputs.append('%fcm [%fs]'%(m*100.0,t))
                        print('test_sampling: ' + config['test_sampling_time'][test_ti]+'\t', end='')
                        print('\t\t'.join(outputs))
                    print('\n')

class CSVOutput(Output):

    def __init__(self, outpath, prjname):
        self.outpath = outpath
        self.prjname = prjname

    def output(self, config, results):
        for task_i in range(len(config['tasks'])):
            fname = os.path.join(self.outpath, self.prjname+config['tasks'][task_i].name+'.csv')
            with open(fname,'w') as f:
                f.write(', '.join([' ']+[m.name+'('+e+' sampling:'+t+')' for m in config['methods'] for e in [a.name for a in config['evaluations']] for t in config['sampling_time']])+'\n')

                outputs = []
                for method_i in range(len(config['methods'])):
                    for ei in range(len(config['evaluations'])):
                        for sti in range(len(config['sampling_time'])):
                            m = results[task_i, method_i, ei, sti].mean()
                            t = results[task_i, method_i, -1, sti]
                            outputs.append('%fcm [%fs]'%(m*100.0,t))
                f.write(','.join(outputs)+'\n')
                f.close()

class H5Output(Output):

    def __init__(self, outpath, prjname):
        self.fname = os.path.join(outpath, prjname+'.h5')

    def output(self, config, results):
            try:
                import h5py
                f = h5py.File(self.fname,'w')
                d = f.create_dataset('results',results.shape, dtype=results.dtype)
                d[:] = results
                f.close()
            except:
                raise 'Fails to write the parameters into a HDF5 file!'
