#!/usr/bin/env python

import csv
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
import pickle


def save_object(obj, filename):
    with open(filename, 'wb') as output:
        pickle.dump(obj, output, pickle.HIGHEST_PROTOCOL)

def open_object(filename):
    with open(filename, 'rb') as input:
        return pickle.load(input)

def func(x):
    return (x-3)*(x-5)*(x-7)+85

class ThreeData:

    def __init__(self):
        self.atime = [] #absolute time
        self.time = []
        self.delta = []
        self.t=[]
        self.data=[]
        self.cov=[]
        self.var=[]
	
    def f(self):
        return 'hello world'

    def readData(self,filename, cov=False):
	
        for row in csv.reader(open(filename, 'r'), delimiter=' ', quotechar='|'):
            #print row
            self.atime.append(float(row[0])/1000000) #absolute time
            self.data.append(np.array([float(row[1]), float(row[2]), float(row[3])]))
            if False != cov:
                matrix = np.array([[float(row[4]), float(row[5]), float(row[6])],
                    [float(row[7]), float(row[8]), float(row[9])],
                    [float(row[10]), float(row[11]), float(row[12])]])
                self.cov.append(matrix)

        atime = self.atime
        self.time.append(0.00)
        for i in range(0,len(atime)-1):
            tbody = float(atime[i+1]) - float(atime[i])
            self.delta.append(tbody)
            tbody = float(atime[i+1]) - float(atime[0])
            self.time.append(tbody)

        self.t = mean(self.delta) * np.r_[0:len(self.atime)]
	
    def eigenValues(self):
        #Eigen values are the axis of the ellipsoid
        for i in range(0,len(self.cov)):
            self.var.append(linalg.eigvals(self.cov[i]))

    def plot_axis(self, fign=1, axis=0, cov=False, levelconf=1, grid=False, linecolor=[1,0,0]):
	
        values = []
        sdmax=[]
        sdmin=[]
        for i in range(0,len(self.data)):
            values.append(self.data[i][axis])
            if False != cov:
                sdmax.append(values[i]+(levelconf*sqrt(self.var[i][axis])))
                sdmin.append(values[i]-(levelconf*sqrt(self.var[i][axis])))
            #print i
            #print values[i]

        #print len(values)
        plt.figure(fign)
        plt.plot(self.t, values, '-o', label="X axis", color=linecolor)

        if False != cov:
            plt.fill_between(self.t, sdmax, sdmin, color=linecolor)

        if False != grid:
            plt.grid(True)

        plt.show(block=False)

    def plot_axis2(self, fign=1, axis=0, cov=False, levelconf=1, grid=False, linecolor=[1,0,0]):
	
        values = []
        sdmax=[]
        sdmin=[]
        for i in range(0,len(self.data)):
            values.append(self.data[i][axis])
            if False != cov:
                sdmax.append(values[i]+(levelconf*sqrt(self.var[i][axis])))
                sdmin.append(values[i]-(levelconf*sqrt(self.var[i][axis])))
            #print i
            #print values[i]
        #print len(values)
        plt.figure(fign)
        plt.plot(self.t, values, '-o', label="X axis", color=linecolor)

        if False != cov:
            plt.plot(self.t, sdmax, color=[0,0,0], linestyle='--')
            plt.plot(self.t, sdmin, color=[0,0,0], linestyle='--')

        if False != grid:
            plt.grid(True)

        plt.show(block=False)

    def plot_errorbars(self, fign=1, axis=0, cov=False, levelconf=1, grid=False, linecolor=[1,0,0]):
	
        values = []
        error=[]
        for i in range(0,len(self.data)):
            values.append(self.data[i][axis])
            if False != cov:
                error.append((levelconf*sqrt(self.var[i][axis])))
            else:
                error.append(0)

        #print len(values)
        plt.figure(fign)
        plt.errorbar(self.t, values,
           yerr=error,
           marker='D',
           color='k',
           ecolor='r',
           lw=2,
           markerfacecolor=linecolor,
           capsize=0,
           linestyle='-')	

        if False != grid:
            plt.grid(True)

        plt.show(block=False)

    def getAxis(self, axis=0):
        values = []
        for i in range(0,len(self.data)):
            values.append(self.data[i][axis])

        return values

    def getCov(self, axis=0):
        values = []
        for i in range(0,len(self.data)):
                values.append(self.cov[i][0:axis+1, 0:axis+1])

        return values

    def getStd(self, axis=0, levelconf=1):
        values = []
        for i in range(0,len(self.data)):
            values.append((levelconf*sqrt(self.var[i][axis])))
        return values

    def getStdMax(self, axis=0, levelconf=1):
        values = []
        sdmax=[]
        for i in range(0,len(self.data)):
            values.append(self.data[i][axis])
            sdmax.append(values[i]+(levelconf*sqrt(self.var[i][axis])))

        return sdmax

    def getStdMin(self, axis=0, levelconf=1):
        values = []
        sdmin=[]
        for i in range(0,len(self.data)):
            values.append(self.data[i][axis])
            sdmin.append(values[i]-(levelconf*sqrt(self.var[i][axis])))

        return sdmin

class QuaternionData:

    def __init__(self):
        self.atime = [] #absolute time
        self.time = []
        self.delta = []
        self.t=[]
        self.data=[]
        self.cov=[]
        self.var=[]
	
    def f(self):
        return 'hello world'

    def readData(self,filename, cov=False):
	
        for row in csv.reader(open(filename, 'r'), delimiter=' ', quotechar='|'):
            #print row
            self.atime.append(float(row[0])/1000000) #absolute time
            self.data.append(quat.quaternion([float(row[4]), float(row[1]), float(row[2]), float(row[3])]))

            if False != cov:
                matrix = np.array([[float(row[5]), float(row[6]), float(row[7])],
                        [float(row[8]), float(row[9]), float(row[10])],
                        [float(row[11]), float(row[12]), float(row[13])]])
                self.cov.append(matrix)
		

        atime = self.atime
        self.time.append(0.00)

        for i in range(0,len(atime)-1):
            tbody = float(atime[i+1]) - float(atime[i])
            self.delta.append(tbody)
            tbody = float(atime[i+1]) - float(atime[0])
            self.time.append(tbody)


        self.t = mean(self.delta) * r_[0:len(self.atime)]
	
    def eigenValues(self):
        #Eigen values are the axis of the ellipsoid
        for i in range(0,len(self.cov)):
	        self.var.append(linalg.eigvals(self.cov[i]))

    def plot_euler(self, fign=1, axis=0, cov=False, levelconf=1, grid=False, linecolor=[1,0,0]):
	
        values = []
        sdmax=[]
        sdmin=[]
        for i in range(0,len(self.data)):
            values.append(self.data[i].toEuler()[axis])
            if False != cov:
                sdmax.append(values[i]+(levelconf*sqrt(self.var[i][axis])))
                sdmin.append(values[i]-(levelconf*sqrt(self.var[i][axis])))
            #print i
            #print values[i]

        #print len(values)
        plt.figure(fign)
        plt.plot(self.t, values, '-o', color=linecolor)

        if False != cov:
            plt.fill_between(self.t, sdmax, sdmin, color=linecolor)

        if False != grid:
            plt.grid(True)

        plt.show(block=False)

    #Euler in [Yaw, Pitch and Roll]
    def getEuler(self, axis):
        values = []
        for i in range(0,len(self.data)):
            values.append(self.data[i].toEuler()[axis])

        return values

    def getCov(self, axis=0):
        values = []
        for i in range(0,len(self.data)):
                values.append(self.cov[i][0:axis+1, 0:axis+1])

        return values

    def getStd(self, axis=0, levelconf=1):
        values = []
        for i in range(0,len(self.data)):
            values.append((levelconf*sqrt(self.var[i][axis])))

        return values

    def getStdMax(self, axis=0, levelconf=1):
        values = []
        sdmax=[]
        for i in range(0,len(self.data)):
            values.append(self.data[i].toEuler()[axis])
            sdmax.append(values[i]+(levelconf*sqrt(self.var[i][axis])))

        return sdmax

    def getStdMin(self, axis=0, levelconf=1):
        values = []
        sdmin=[]
        for i in range(0,len(self.data)):
            values.append(self.data[i].toEuler()[axis])
            sdmin.append(values[i]-(levelconf*sqrt(self.var[i][axis])))

        return sdmin


class OneData:

    def __init__(self):
        self.time = []
        self.delta = []
        self.t=[]
        self.data=[]
        self.cov=[]
        self.var=[]
	
    def f(self):
        return 'hello world'

    def readData(self,filename, cov=False):
        for row in csv.reader(open(filename, 'r'), delimiter=' ', quotechar='|'):
            #print row
            self.time.append(float(row[0])/1000000)
            self.data.append(np.array([float(row[1])]))

            if False != cov:
                matrix = np.array([[float(row[2])]])
                self.cov.append(matrix)
                self.var.append(sqrt(matrix))
		

        time = self.time
        for i in range(0,len(time)-1):
            tbody = float(time[i+1]) - float(time[i])
            self.delta.append(tbody)

        self.t = mean(self.delta) * r_[0:len(self.time)]
	
    def plot_axis(self, fign=1, axis=0, cov=False, levelconf=1, grid=False, linecolor=[1,0,0]):
	
        values = []
        sdmax=[]
        sdmin=[]
        for i in range(0,len(self.data)):
            values.append(self.data[i][axis])
            if False != cov:
                sdmax.append(values[i]+(levelconf*sqrt(self.var[i][axis])))
                sdmin.append(values[i]-(levelconf*sqrt(self.var[i][axis])))
            #print i
            #print values[i]

        #print len(values)
        plt.figure(fign)
        plt.plot(self.t, values, '-o', label="X axis", color=linecolor)

        if False != cov:
            plt.fill_between(self.t, sdmax, sdmin, color=linecolor)

        if False != grid:
            plt.grid(True)

        plt.show(block=False)

