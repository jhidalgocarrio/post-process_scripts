# Copyright (c) 2015, Zhenwen Dai
# Licensed under the BSD 3-clause license (see LICENSE.txt)

from __future__ import print_function
import abc
import os
import numpy as np

class Output(object):
    __metaclass__ = abc.ABCMeta
    
    @abc.abstractmethod
    def output(self, config, results):
        """Return the test data: training data and labels"""
        return None

class ScreenOutput(Output):

    def output(self, config, results):
        print('='*20+' REPORT '+'='*20)
        print('\t\t\t'.join([' ']+[m.name+'('+e+' sampling:'+t+')' for m in config['methods'] for e in [a.name for a in config['evaluations']] for t in config['sampling_time']]))

        for task_i in range(len(config['tasks'])):
            print(config['tasks'][task_i].name+'\t', end='')

            outputs = []
            for method_i in range(len(config['methods'])):
                for ei in range(len(config['evaluations'])):
                    for sti in range(len(config['sampling_time'])):
                        m = results[task_i, method_i, ei, sti].mean()
                        t = results[task_i, method_i, -1, sti]
                        outputs.append('%fcm [%fs]'%(m*100.0,t))
            print('\t'.join(outputs))

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
