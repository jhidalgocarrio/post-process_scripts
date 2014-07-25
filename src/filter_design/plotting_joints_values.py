#!/usr/bin/python
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt


#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140605_pink_odometry/20140605-1731/data/left_passive_position.data', 'rb'), delimiter=' ', quotechar='|')
spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140605_pink_odometry/20140605-1731/data/left_passive_position_localization_frontend.data', 'rb'), delimiter=' ', quotechar='|')

#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140603_passive_joints/20140604-1134/left_passive_position.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140603_passive_joints/20140604-1134/left_passive_position_localization_frontend.data', 'rb'), delimiter=' ', quotechar='|')

#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140603_passive_joints/20140603-1856/left_passive_position.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140603_passive_joints/20140603-1856/left_passive_position_localization_frontend.data', 'rb'), delimiter=' ', quotechar='|')

#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140522-2014_new_transformer/20140529-1903/left_passive_position.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140522-2014_new_transformer/20140529-1903/left_passive_position_localization_frontend.data', 'rb'), delimiter=' ', quotechar='|')

time=[]
joint=[]

for row in spamReader:
    #print row
    time.append(float(row[0])/1000000)
    joint.append(float(row[1]))

delta = []
for i in range(0,len(time)-1):
    #print time[i]
    t = float(time[i+1]) - float(time[i])
    delta.append(t)

delta_t = mean(delta)
sample_rate = 1/delta_t
t = delta_t * r_[0:len(time)]
#################
### GRAPHICS  ###
#################
plt.figure(1)
plot(t,joint, '-o', label="Passive Joint", color='green')
plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Angle [$rad$]', fontsize=35, fontweight='bold')
plt.legend(prop={'size':25})
plt.show(block=False)

plt.figure(2)
jointdeg = [x * 180.00/math.pi for x in joint]
plot(t,jointdeg, '-o', label="Passive Joint")
plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Angle [${}^\circ$]', fontsize=35, fontweight='bold')
plt.legend(prop={'size':25})
plt.show(block=False)

meanjoint = []
stdjointpos = []
stdjointneg = []
numbermean = mean(joint)
numberstd = std(joint)
numbervar = var(joint)
for i in range(0,len(time)):
    meanjoint.append(numbermean)
    stdjointpos.append(numbermean + numberstd)
    stdjointneg.append(numbermean - numberstd)

plt.figure(3)
plot(t,joint, '-o', label="Passive Joint", color='green')
plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Angle [$rad$]', fontsize=35, fontweight='bold')
plot(t,meanjoint, label="Mean value", color='red')
plot(t,stdjointpos, label="Std (+)", color='black')
plot(t,stdjointneg, label="Std (-)", color='black')
plt.legend(prop={'size':25})
plt.show(block=False)

