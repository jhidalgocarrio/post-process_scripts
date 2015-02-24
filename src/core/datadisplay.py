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

        self.delta.append(self.delta[len(self.delta)-1])
        self.t = mean(self.delta) * np.r_[0:len(self.atime)]

        # Convert to np array
        self.atime = np.asarray(self.atime)
        self.time = np.asarray(self.time)
        self.delta = np.asarray(self.delta)
        self.t = np.asarray(self.t)
        self.data = np.asarray(self.data)
        self.cov = np.asarray(self.cov)

    def covSymmetry(self):
        for i in range(0,len(self.cov)):
            self.cov[i] = 0.5 * (self.cov[i] + self.cov[i].T)
	
    def eigenValues(self):
        #Eigen values are the axis of the ellipsoid
        for i in range(0,len(self.cov)):
            self.var.append(np.real(linalg.eigvals(self.cov[i]))[::-1]) #real number and in reverse order

        self.var = np.asarray(self.var)

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

    def getCov(self):
        return np.array(self.cov[:,axis])

    def getStd(self, axis=0, levelconf=1):
        return np.array(levelconf*np.sqrt(self.var[:,axis]))

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

    def delete(self, index_to_remove):
        """Delete internal data from the index specified in temindex """

        indexes = np.setdiff1d(xrange(len(self.data)), index_to_remove)

        self.atime = self.atime[indexes]
        self.time = self.time[indexes]
        self.delta = self.delta[indexes]
        self.t = self.t[indexes]
        self.data = self.data[indexes]
        if len(self.cov) > 0:
            self.cov = self.cov[indexes]
        if len(self.var) > 0:
            self.var = self.var[indexes]

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

        self.delta.append(self.delta[len(self.delta)-1])
        self.t = mean(self.delta) * r_[0:len(self.atime)]
	
        # Convert to np array
        self.atime = np.asarray(self.atime)
        self.time = np.asarray(self.time)
        self.delta = np.asarray(self.delta)
        self.t = np.asarray(self.t)
        self.cov = np.asarray(self.cov)

    def covSymmetry(self):
        for i in range(0,len(self.cov)):
            self.cov[i] = 0.5 * (self.cov[i] + self.cov[i].T)

    def eigenValues(self):
        #Eigen values are the axis of the ellipsoid
        for i in range(0,len(self.cov)):
            self.var.append(np.real(linalg.eigvals(self.cov[i]))[::-1]) #real number and in reverse order
        self.var = np.asarray(self.var)

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

    def getCov(self):
        return np.array(self.cov[:,axis])

    def getStd(self, axis=0, levelconf=1):
        return np.array(levelconf*np.sqrt(self.var[:,axis]))

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

    def delete(self, index_to_remove):
        """Delete internal data from the index specified in temindex """

        indexes = np.setdiff1d(xrange(len(self.data)), index_to_remove)

        self.atime = self.atime[indexes]
        self.time = self.time[indexes]
        self.delta = self.delta[indexes]
        self.t = self.t[indexes]

        #convert quaternion to array and remove outliers
        dataq = np.asarray(self.data)
        dataq = dataq[indexes]

        #Get back to list of quaternions
        self.data = []
        for q in dataq:
            self.data.append(quat.quaternion([float(q[0]), float(q[1]), float(q[2]), float(q[3])]))

        if len(self.cov) > 0:
            self.cov = self.cov[indexes]
        if len(self.var) > 0:
            self.var = self.var[indexes]



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

        self.delta.append(self.delta[len(self.delta)-1])
        self.t = mean(self.delta) * r_[0:len(self.time)]

        # Convert to np array
        self.atime = np.asarray(self.atime)
        self.time = np.asarray(self.time)
        self.delta = np.asarray(self.delta)
        self.t = np.asarray(self.t)
        self.data = np.asarray(self.data)
        self.cov = np.asarray(self.cov)
        self.var = np.asarray(self.var)

	
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

    def delete(self, index_to_remove):
        """Delete internal data from the index specified in temindex """

        indexes = np.setdiff1d(xrange(len(self.data)), index_to_remove)

        self.atime = self.atime[indexes]
        self.time = self.time[indexes]
        self.delta = self.delta[indexes]
        self.t = self.t[indexes]
        self.data = self.data[indexes]
        if len(self.cov) > 0:
            self.cov = self.cov[indexes]
        if len(self.var) > 0:
            self.var = self.var[indexes]


