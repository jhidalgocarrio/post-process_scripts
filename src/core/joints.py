
import csv, scipy, sys
from pylab import *
import numpy as np

class JointState:

    def __init__(self, position=None, speed=None):
        if position is None:
            self.position = np.nan
        if speed is None:
            self.speed = np.nan

    def f(self):
        return 'hello world JointState'

class Joints:
    def __init__(self, names=None):
        self.atime = [] #absolute time
        self.time = []
        self.delta = []
        self.t=[]
        self.names=[]
        self.position=[]
        self.speed=[]
        if names is not None:
            self.names = names
        else:
            raise ValueError("Joints names cannot be empty. You need to specify the names!")

        return

    def f(self):
        return 'Hello world this is Joints'


    def readData(self, filenameposition, filenamespeed):

        for row in csv.reader(open(filenameposition, 'r'), delimiter=' ', quotechar='|'):
            #print (row)
            element_row = np.array([])
            for item in row:
                element_row = np.append(element_row, item)

            self.atime.append(float(element_row[0])/1000000) #absolute time
            element_row = np.delete(element_row, 0)

            if (size(element_row) is not size(self.names)):
                raise ValueError("Row elements and names have different size.")

            joints_row = np.array([])
            for i in range(0, len(element_row)):
                joints_row = np.append(joints_row, element_row[i])
            self.position.append(joints_row)

        for row in csv.reader(open(filenamespeed, 'r'), delimiter=' ', quotechar='|'):
            #print (row)
            element_row = np.array([])
            for item in row:
                element_row = np.append(element_row, item)

            element_row = np.delete(element_row, 0)

            if (size(element_row) is not size(self.names)):
                raise ValueError("Row elements and names have different size.")

            joints_row = np.array([])
            for i in range(0, len(element_row)):
                joints_row = np.append(joints_row, element_row[i])
            self.speed.append(joints_row)

        if (len(self.position) != len(self.speed)):
            raise ValueError("Number of position and speed values must be the same")

        atime = self.atime
        self.time.append(0.00)
        for i in range(0,len(atime)-1):
            tbody = float(atime[i+1]) - float(atime[i])
            self.delta.append(tbody)
            tbody = float(atime[i+1]) - float(atime[0])
            self.time.append(tbody)
        self.t = mean(self.delta) * r_[0:len(self.atime)]

