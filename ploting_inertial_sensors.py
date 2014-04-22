#!/usr/bin/python
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt

###########
##  ACC  ##
###########

#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_acc_262bnw_500hz.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_acc_16bnw_500hz.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_acc_16bnw_250hz.data', 'rb'), delimiter=' ', quotechar='|')
spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_acc_33bnw_125hz.data', 'rb'), delimiter=' ', quotechar='|')

time=[]
accx=[]
accy=[]
accz=[]

for row in spamReader:
    #print row
    time.append(float(row[0])/1000000)
    accx.append(float(row[1]))
    accy.append(float(row[2]))
    accz.append(float(row[3]))


delta = []
for i in range(0,len(time)-1):
    #print time[i]
    dt = float(time[i+1]) - float(time[i])
    delta.append(dt)

delta_t = mean(delta)
sample_rate = 1/delta_t
t = delta_t * r_[0:len(time)]

meanaccx = []
stdaccxpos = []
stdaccxneg = []
numbermean = mean(accx)
numberstd = std(accx)
numbervar = var(accx)
for i in range(0,len(time)):
    meanaccx.append(numbermean)
    stdaccxpos.append(numbermean + numberstd)
    stdaccxneg.append(numbermean - numberstd)

meanaccy = []
stdaccypos = []
stdaccyneg = []
numbermean = mean(accy)
numberstd = std(accy)
numbervar = var(accy)
for i in range(0,len(time)):
    meanaccy.append(numbermean)
    stdaccypos.append(numbermean + numberstd)
    stdaccyneg.append(numbermean - numberstd)

meanaccz = []
stdacczpos = []
stdacczneg = []
numbermean = mean(accz)
numberstd = std(accz)
numbervar = var(accz)
for i in range(0,len(time)):
    meanaccz.append(numbermean)
    stdacczpos.append(numbermean + numberstd)
    stdacczneg.append(numbermean - numberstd)

plt.figure(4)
plot(t,accx, label="X axis", color='blue')
plot(t,meanaccx, label="Mean value", color='black')
plot(t,stdaccxpos, label="Std (+)", color='grey')
plot(t,stdaccxneg, label="Std (-)", color='grey')
plt.show(block=False)

plt.figure(5)
plot(t,accy, label="Y axis", color='green')
plot(t,meanaccy, label="Mean value", color='black')
plot(t,stdaccypos, label="Std (+)", color='grey')
plot(t,stdaccyneg, label="Std (-)", color='grey')
plt.show(block=False)

plt.figure(6)
plot(t,accz, label="Z axis", color='blue')
plot(t,meanaccz, label="Mean value", color='black')
plot(t,stdacczpos, label="Std (+)", color='grey')
plot(t,stdacczneg, label="Std (-)", color='grey')
plt.show(block=False)

###########
## GYROS ##
###########

spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_gyro_262bnw_500hz.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_gyro_16bnw_500hz.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_gyro_16bnw_250hz.data', 'rb'), delimiter=' ', quotechar='|')

time=[]
gyrox=[]
gyroy=[]
gyroz=[]

for row in spamReader:
    #print row
    time.append(float(row[0])/1000000)
    gyrox.append(float(row[1]))
    gyroy.append(float(row[2]))
    gyroz.append(float(row[3]))


delta = []
for i in range(0,len(time)-1):
    #print time[i]
    dt = float(time[i+1]) - float(time[i])
    delta.append(dt)

delta_t = mean(delta)    
sample_rate = 1/delta_t
t = delta_t * r_[0:len(time)]

meangyrox = []
stdgyroxpos = []
stdgyroxneg = []
numbermean = mean(gyrox)
numberstd = std(gyrox)
numbervar = var(gyrox)
for i in range(0,len(time)):
    meangyrox.append(numbermean)
    stdgyroxpos.append(numbermean + numberstd)
    stdgyroxneg.append(numbermean - numberstd)

meangyroy = []
stdgyroypos = []
stdgyroyneg = []
numbermean = mean(gyroy)
numberstd = std(gyroy)
numbervar = var(gyroy)
for i in range(0,len(time)):
    meangyroy.append(numbermean)
    stdgyroypos.append(numbermean + numberstd)
    stdgyroyneg.append(numbermean - numberstd)

meangyroz = []
stdgyrozpos = []
stdgyrozneg = []
numbermean = mean(gyroz)
numberstd = std(gyroz)
numbervar = var(gyroz)
for i in range(0,len(time)):
    meangyroz.append(numbermean)
    stdgyrozpos.append(numbermean + numberstd)
    stdgyrozneg.append(numbermean - numberstd)

plt.figure(4)
plot(t,gyrox, label="X axis", color='cyan')
plot(t,meangyrox, label="Mean value", color='black')
plot(t,stdgyroxpos, label="Std (+)", color='grey')
plot(t,stdgyroxneg, label="Std (-)", color='grey')
plt.show(block=False)

plt.figure(5)
plot(t,gyroy, label="Y axis", color='green')
plot(t,meangyroy, label="Mean value", color='black')
plot(t,stdgyroypos, label="Std (+)", color='grey')
plot(t,stdgyroyneg, label="Std (-)", color='grey')
plt.show(block=False)

plt.figure(6)
plot(t,gyroz, label="Z axis", color='blue')
plot(t,meangyroz, label="Mean value", color='black')
plot(t,stdgyrozpos, label="Std (+)", color='grey')
plot(t,stdgyrozneg, label="Std (-)", color='grey')
plt.show(block=False)

###########
##  INC  ##
###########

#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_inc_262bnw_500hz.data', 'rb'), delimiter=' ', quotechar='|')
spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_inc_16bnw_500hz.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_inc_16bnw_250hz.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test/stim300_inc_33bnw_125hz.data', 'rb'), delimiter=' ', quotechar='|')

time=[]
incx=[]
incy=[]
incz=[]

for row in spamReader:
    #print row
    time.append(float(row[0])/1000000)
    incx.append(float(row[1]))
    incy.append(float(row[2]))
    incz.append(float(row[3]))


delta = []
for i in range(0,len(time)-1):
    #print time[i]
    dt = float(time[i+1]) - float(time[i])
    delta.append(dt)

delta_t = mean(delta)    
sample_rate = 1/delta_t
t = delta_t * r_[0:len(time)]

meanincx = []
stdincxpos = []
stdincxneg = []
numbermean = mean(incx)
numberstd = std(incx)
numbervar = var(incx)
for i in range(0,len(time)):
    meanincx.append(numbermean)
    stdincxpos.append(numbermean + numberstd)
    stdincxneg.append(numbermean - numberstd)

meanincy = []
stdincypos = []
stdincyneg = []
numbermean = mean(incy)
numberstd = std(incy)
numbervar = var(incy)
for i in range(0,len(time)):
    meanincy.append(numbermean)
    stdincypos.append(numbermean + numberstd)
    stdincyneg.append(numbermean - numberstd)

meanincz = []
stdinczpos = []
stdinczneg = []
numbermean = mean(incz)
numberstd = std(incz)
numbervar = var(incz)
for i in range(0,len(time)):
    meanincz.append(numbermean)
    stdinczpos.append(numbermean + numberstd)
    stdinczneg.append(numbermean - numberstd)

plt.figure(4)
plot(t,incx, label="X axis", color='cyan')
plot(t,meanincx, label="Mean value", color='black')
plot(t,stdincxpos, label="Std (+)", color='grey')
plot(t,stdincxneg, label="Std (-)", color='grey')
plt.show(block=False)

plt.figure(5)
plot(t,incy, label="Y axis", color='green')
plot(t,meanincy, label="Mean value", color='black')
plot(t,stdincypos, label="Std (+)", color='grey')
plot(t,stdincyneg, label="Std (-)", color='grey')
plt.show(block=False)

plt.figure(6)
plot(t,incz, label="Z axis", color='blue')
plot(t,meanincz, label="Mean value", color='black')
plot(t,stdinczpos, label="Std (+)", color='grey')
plot(t,stdinczneg, label="Std (-)", color='grey')
plt.show(block=False)

