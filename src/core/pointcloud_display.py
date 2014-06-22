#!/usr/bin/env python

import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat

class PointCloud:

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

    def readData(self, filename1, filename2, cov=False):
        #Get the positions and the time
	for row in csv.reader(open(filename1, 'rb'), delimiter=' ', quotechar='|'):
	    #print row
	    self.atime.append(float(row[0])/1000000) #absolute time
            #Get all the 3D-points in a row
            j=0
            for i in range(int(math.ceil((len(row)-1)/3.0))): #-1 because it also includes the timestamp
                #print int(math.ceil((len(row)-1)/3.0))
                self.data.append(np.array([float(row[int(j+1)]), float(row[int(j+2)]), float(row[int(j+3)])]))
                j=j+3

        if False != cov:
            for row in csv.reader(open(filename2, 'rb'), delimiter=' ', quotechar='|'):
                j=0
                for i in range(int(math.ceil(float(row[1])/3.0))):
                    matrix = np.array([[float(row[j+2]), float(row[j+3]), float(row[j+4])],
                        [float(row[j+5]), float(row[j+6]), float(row[j+7])],
                        [float(row[j+8]), float(row[j+9]), float(row[j+10])]])
                    self.cov.append(matrix)
                    j=j+9
		
	atime = self.atime
        self.time.append(0.00)
	for i in range(0,len(atime)-1):
	    tbody = float(atime[i+1]) - float(atime[i])
	    self.delta.append(tbody)
            tbody = float(atime[i+1]) - float(atime[0])
            self.time.append(tbody)


#	self.t = mean(self.delta) * r_[0:len(self.atime)]
	
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


