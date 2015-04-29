###############################################
# Post Processing R Script to compute
# the Allan variance over the STIM300 5g
# Javier Hidalgo Carrió
# DFKI-RIC ESTEC March 2014
###############################################

#libraries
library(allanvar)

# You may change this path
setwd ("/home/jhidalgocarrio/exoter/development/post-process_data/20140325_stim300_test")

#############
# TEST Temp
#############
values <- read.table ("", sep=" ")

names(values) = c("time", "tempx", "tempy", "tempz")

#Temp is in kelvins

values$tempx = values$tempx - 273.15
values$tempy = values$tempy - 273.15
values$tempz = values$tempz - 273.15

values$tempx <- values$tempx[1:length(values$tempx)-1]
values$tempy <- values$tempy[1:length(values$tempy)-1]
values$tempz <- values$tempz[1:length(values$tempz)-1]

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[3]-values$time[2])/1000000

#############
# Gyroscopes
#############
gvalues <- read.table ("stim300_driver_test.allan.time_gyro.data", sep=" ")

names(gvalues) = c("time", "gyrox", "gyroy", "gyroz")

str(gvalues)


avalues <- read.table ("stim300_driver_test.allan.time_acc.data", sep=" ")

names(avalues) = c("time", "accx", "accy", "accz")

str(avalues)


t = delta * seq(1:length(values$time))
plot (t, values$tempx, col="red")
lines (t, values$tempy, col="green")
lines (t, values$tempz, col="blue")

plot (values$tempx[1:length(values$tempx)-1], gvalues$gyrox, col="red")
plot (values$tempx[1:length(values$tempx)-1], avalues$accx, col="red")

plot (values$tempy[1:length(values$tempy)-1], gvalues$gyroy, col="green")
plot (values$tempy[1:length(values$tempy)-1], avalues$accy, col="green")

plot (values$tempz[1:length(values$tempz)-1], gvalues$gyroz, col="blue")
plot (values$tempz[1:length(values$tempz)-1], avalues$accz, col="blue")


