
import csv, sys
from pylab import *
import numpy as np

class JointState:

    def __init__(self, position=None, speed=None):
        if position is None:
            self.position = np.nan
        else:
            self.position = position
        if speed is None:
            self.speed = np.nan
        else:
            self.speed = speed

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

    def getJoint(self, jointname = None):

        if jointname is None:
            raise ValueError("You need to specify the name of the joint")
        if len(self.names) == 0:
            raise ValueError("Names is empty!!")

        jointidx = self.names.index(jointname)

        joint_array = np.array([])
        for i in range(0, len(self.position)):
            joint_array = np.append(joint_array, JointState(self.position[i][jointidx], self.speed[i][jointidx]))

        return joint_array


    def getPosition(self, jointname = None):

        if jointname is None:
            raise ValueError("You need to specify the name of the joint")
        if len(self.names) == 0:
            raise ValueError("Names is empty!!")

        jointidx = self.names.index(jointname)

        position_array = np.array([])
        for i in range(0, len(self.position)):
            position_array = np.append(position_array, self.position[i][jointidx])

        return position_array

    def getSpeed(self, jointname = None):

        if jointname is None:
            raise ValueError("You need to specify the name of the joint")
        if len(self.names) == 0:
            raise ValueError("Names is empty!!")

        jointidx = self.names.index(jointname)

        speed_array = np.array([])
        for i in range(0, len(self.speed)):
            speed_array = np.append(speed_array, self.speed[i][jointidx])

        return speed_array


