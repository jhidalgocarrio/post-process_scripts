#!/usr/bin/env python

import csv
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
import pickle
import os

############################################
## HELPER METHOD FOR SAVING AND OPENING   ##
############################################

def save_object(obj, filename, mode='wb'):
    with open(os.path.expanduser(filename), mode) as output:
        pickle.dump(obj, output, pickle.HIGHEST_PROTOCOL)

def open_object(filename):
    with open(os.path.expanduser(filename), 'rb') as input:
        return pickle.load(input)

def func(x):
    return (x-3)*(x-5)*(x-7)+85

#############################################
## AVERAGE FILTER BY SPLITTING INPUT TEST  ##
#############################################
def input_reduction (input_array = None, number_blocks = 0.0):
    if input_array is None:
        raise ValueError("Input cannot be None")
    if number_blocks is 0.0:
        raise ValueError("Cannot split array with zero number of blocks")

    input_array_shape = 0
    if len(input_array.shape) == 2:
        input_array_shape = input_array.shape[1]
    else:
        input_array_shape = 1

    new_input = np.ndarray(shape = (number_blocks, input_array_shape))
    std_input = np.ndarray(shape = (number_blocks, input_array_shape))

    for i in range(input_array_shape):
        if input_array_shape > 1:
            split_array = np.array_split(input_array[:,i], number_blocks)
        else:
            split_array = np.array_split(input_array, number_blocks)

        mean_array = np.ndarray(len(split_array), dtype=double)
        std_array = np.ndarray(len(split_array), dtype=double)

        for j in range(len(split_array)):
            mean_array[j] = mean(split_array[j])
            std_array[j] = std(split_array[j])

        if input_array_shape > 1:
            new_input[:,i] = mean_array
            std_input[:,i] = std_array
        else:
            new_input = mean_array
            std_input = std_array

    return new_input, std_input
#####################################################

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
	
        for row in csv.reader(open(os.path.expanduser(filename), 'r'), delimiter=' ', quotechar='|'):
            #print row
            self.atime.append(float(row[0])/1000000.00) #absolute time
            self.data.append(np.array([float(row[1]), float(row[2]), float(row[3])]))
            if False != cov:
                # pocolog matrix (ruby) are organized by rows
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

        if len(self.delta) > 1:
            self.delta.append(self.delta[len(self.delta)-1])
            self.t = mean(self.delta) * r_[0:len(self.atime)]

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
            self.var.append(np.real(linalg.eigvals(self.cov[i]))) #real number

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

    def readData(self,filename, angle_axis=False, cov=False):
	
        for row in csv.reader(open(os.path.expanduser(filename), 'r'), delimiter=' ', quotechar='|'):
            #print row
            self.atime.append(float(row[0])/1000000.00) #absolute time
            if False != angle_axis:
                self.data.append(quat.quaternion.fromAngleAxis(float(row[4]), [float(row[1]), float(row[2]), float(row[3])]))
            else:
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

        if len(self.delta) > 1:
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
            self.var.append(np.real(linalg.eigvals(self.cov[i]))) #real number
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
        for row in csv.reader(open(os.path.expanduser(filename), 'r'), delimiter=' ', quotechar='|'):
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

## A classic Hash table
#
class HashTable:
    def __init__(self, _size):
        self.size = _size
        self.slots = [None] * self.size
        self.data = [None] * self.size

    def put(self,key,data):
        hashvalue = self.hashfunction(key,len(self.slots))

        if self.slots[hashvalue] == None:
            self.slots[hashvalue] = key
            self.data[hashvalue] = data
        else:
            if self.slots[hashvalue] == key:
                self.data[hashvalue] = data  #replace
            else:
                nextslot = self.rehash(hashvalue,len(self.slots))
                while self.slots[nextslot] != None and self.slots[nextslot] != key:
                    nextslot = self.rehash(nextslot,len(self.slots))
                if self.slots[nextslot] == None:
                    self.slots[nextslot]=key
                    self.data[nextslot]=data
                else:
                    self.data[nextslot] = data #replace

    def hashfunction(self,key,size):
        return key%size
	
    def rehash(self,oldhash,size):
        return (oldhash+1)%size
	
    def get(self,key):
        startslot = self.hashfunction(key,len(self.slots))

        data = None
        stop = False
        found = False
        position = startslot
        while self.slots[position] != None and not found and not stop:
            if self.slots[position] == key:
                found = True
                data = self.data[position]
            else:
                position=self.rehash(position,len(self.slots))
                if position == startslot:
                    stop = True
        return data

    def __getitem__(self,key):
        return self.get(key)

    def __setitem__(self,key,data):
        self.put(key,data)

