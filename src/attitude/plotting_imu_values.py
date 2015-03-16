#!/usr/bin/python
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt

spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/data/20140325_stim300_test/stim300_acc_33bnw_125hz.data', 'rb'), delimiter=' ', quotechar='|')

timeimu=[]
accimux=[]
accimuy=[]
accimuz=[]

for row in spamReader:
    #print row
    timeimu.append(float(row[0])/1000000)
    accimux.append(float(row[1]))
    accimuy.append(float(row[2]))
    accimuz.append(float(row[3]))

deltaimu = []
for i in range(0,len(timeimu)-1):
    #print time[i]
    timu = float(timeimu[i+1]) - float(timeimu[i])
    deltaimu.append(timu)

deltaimu_t = mean(deltaimu)    
sample_rateimu = 1/deltaimu_t
timu = deltaimu_t * r_[0:len(timeimu)]
#################
### GRAPHICS  ###
#################
plt.figure(1)
plot(timu,accimux, '-o', label="x imu acc")
plt.xlabel(r'time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'acc [$m/s^2$]', fontsize=35, fontweight='bold')
plt.legend(prop={'size':25})
plt.show(block=False)


plt.figure(2)
plot(timu,accimuy, '-o', label="y imu acc")
plt.xlabel(r'time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'acc [$m/s^2$]', fontsize=35, fontweight='bold')
plt.legend(prop={'size':25})
plt.show(block=False)

plt.figure(3)
plot(timu,accimuz, '-o', label="z imu acc")
plt.xlabel(r'time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'acc [$m/s^2$]', fontsize=35, fontweight='bold')
plt.legend(prop={'size':25})
plt.show(block=False)

