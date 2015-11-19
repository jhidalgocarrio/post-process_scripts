###############################################
# Post Processing R Script to compute
# the Allan variance over the STIM300 5g
# Javier Hidalgo Carrio
# DFKI-RIC ESTEC March 2014
###############################################

#libraries
library(allanvar)
library(timeSeries)

# You may change this path
setwd ("/home/jhidalgocarrio/exoter/development/data/20140325_stim300_test")

#load("stim300_gyro_16bnw_500hz_analysis.Rdata")
#load("stim300_gyro_16bnw_250hz_analysis.Rdata")
#load("stim300_gyro_262bnw_500hz_analysis.Rdata")

#############
# TEST Gyro
#############
#values <- read.table ("stim300_gyro_16bnw_500hz.data", sep="\t")
#values <- read.table ("stim300_gyro_16bnw_250hz.data", sep=" ")
#values <- read.table ("stim300_gyro_262bnw_500hz.data", sep=" ")
values <- read.table ("stim300_gyro_33bnw_125hz.data", sep=" ")

names(values) = c("time", "gyrox", "gyroy", "gyroz")

str(values)

#Time is in microseconds (10^-6)
delta <- diff(values$time)
delta <- mean(delta)/1000000

########################## X Axis ##########################
stim300Gyro <- ts(as.numeric(array(na.omit(values$gyrox))), deltat=mean(delta))

#Frequency
frequency (stim300Gyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1x <- avar (stim300Gyro@.Data, frequency (stim300Gyro))

########################## Y Axis ##########################
stim300Gyro <- ts(as.numeric(array(na.omit(values$gyroy))), deltat=mean(delta))

#Frequency
frequency (stim300Gyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1y <- avar (stim300Gyro@.Data, frequency (stim300Gyro))

########################## Z Axis ##########################
stim300Gyro <- ts(as.numeric(array(na.omit(values$gyroz))), deltat=mean(delta))

#Frequency
frequency (stim300Gyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1z <- avar (stim300Gyro@.Data, frequency (stim300Gyro))

#### Plotting the results ####
x11()
plot (avgyro1x$time,sqrt(avgyro1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avgyro1y$time,sqrt(avgyro1y$av), col="green", lwd=1)
lines (avgyro1z$time,sqrt(avgyro1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=2.0, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 5e-03, c("GyroscopeX", "GyroscopeY", "GyroscopeZ"),  fill = c("blue", "green", "red"))

#dev.print(png, file="xsens_gyro.png", width=1024, height=768, bg = "white") # To save the x11 device in a png

#### Plotting the results in a file ####
png(filename = "stim300_gyro_allanvar.png", width=2048, height=1536, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avgyro1x$time,sqrt(avgyro1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avgyro1y$time,sqrt(avgyro1y$av), col="green", lwd=1)
lines (avgyro1z$time,sqrt(avgyro1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=2.0, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 5e-03, c("GyroscopeX", "GyroscopeY", "GyroscopeZ"),  fill = c("blue", "green", "red"))
dev.off()

### Delete all values
rm (values, stim300Gyro)

#### Save the result in a R image
#save.image (file = "stim300_gyro_16bnw_500hz_analysis.Rdata")
#save.image (file = "stim300_gyro_16bnw_250hz_analysis.Rdata")
#save.image (file = "stim300_gyro_262bnw_500hz_analysis.Rdata")

#######################################
#### Calculate the values (STIM300 Gyro)#
#######################################
plotav (avgyro1x)

##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 #
approx (x=c(avgyro1x$time[9], avgyro1x$time[10]), y= c(sqrt(avgyro1x$av[9]), sqrt(avgyro1x$av[10])), n=100)
4.023064e-05 #16bnw_500hz
5.421025e-05 #262bnw_500hz
approx (x=c(avgyro1x$time[8], avgyro1x$time[9]), y= c(sqrt(avgyro1x$av[8]), sqrt(avgyro1x$av[9])), n=100)
4.037031e-05 #16bnw_250hz
approx (x=c(avgyro1x$time[7], avgyro1x$time[8]), y= c(sqrt(avgyro1x$av[7]), sqrt(avgyro1x$av[8])), n=100)
4.131546e-05 #33bnw_125hz

#COMPARISON WITH OTHERS
4.320343e-05 #(5 hours test)
4.274725e-05 #STIM300 rad/s/sqrt(Hz) datasheet 
6.349347e-05 #iMAR rad/s/sqrt(Hz) datasheet 
0.0006643596 #XSens rad/s/sqrt(Hz)

approx (x=c(avgyro1y$time[9], avgyro1y$time[10]), y= c(sqrt(avgyro1y$av[9]), sqrt(avgyro1y$av[10])), n=100)
4.274826e-05 #16bnw_500hz
5.818066e-05 #262bnw_500hz
approx (x=c(avgyro1y$time[8], avgyro1y$time[9]), y= c(sqrt(avgyro1y$av[8]), sqrt(avgyro1y$av[9])), n=100)
4.452885e-05 #16bnw_250hz
approx (x=c(avgyro1y$time[7], avgyro1y$time[8]), y= c(sqrt(avgyro1y$av[7]), sqrt(avgyro1y$av[8])), n=100)
4.478284e-05 #33bnw_125hz

#COMPARISON WITH OTHERS
4.175001e-05 #(5 hours test)
4.268059e-05 #STIM300 rad/s/sqrt(Hz)
1.049867e-04 #iMAR rad/s/sqrt(Hz)
0.0006978859 #rad/s/sqrt(Hz)

approx (x=c(avgyro1z$time[9], avgyro1z$time[10]), y= c(sqrt(avgyro1z$av[9]), sqrt(avgyro1z$av[10])), n=100)
3.871778e-05 #16bnw_500hz
5.361688e-05 #262bnw_500hz
approx (x=c(avgyro1z$time[8], avgyro1z$time[9]), y= c(sqrt(avgyro1z$av[8]), sqrt(avgyro1z$av[9])), n=100)
3.985315e-05 #16bnw_250hz
approx (x=c(avgyro1z$time[7], avgyro1z$time[8]), y= c(sqrt(avgyro1z$av[7]), sqrt(avgyro1z$av[8])), n=100)
4.093230e-05 #33bnw_125hz

#COMPARISON WITH OTHERS
4.060973e-05 #(5 hours test)
4.319166e-05 #STIM300 rad/s/sqrt(Hz)
7.936806e-05 #iMAR rad/s/sqrt(Hz)
0.0006390777 #rad/s/sqrt(Hz)

##
#Bias Instability, can be directly obtained by reading the 0 slope line 
##

#Test 1 Bias instability Coeff
sqrt(avgyro1x$av[18])/(sqrt(2*log(2)/pi))
7.051878e-06 #16bnw_500hz
#COMPARISON WITH OTHERS
6.882191e-06 #(5 hours test)
4.697074e-06 #STIM300 rad/s datasheet is 0.5 deg/hr 0.016 deg/hr
1.857284e-06 #iMAR rad/s datasheet
0.0001463489 #XSens rad/s

sqrt(avgyro1y$av[19])/(sqrt(2*log(2)/pi))
4.814562e-06 #16bnw_500hz
#COMPARISON WITH OTHERS
8.479295e-06 #(5 hours test)
8.841014e-06 #STIM300 rad/s
6.456726e-06 #iMAR rad/s
0.000151518 #XSens rad/s

sqrt(avgyro1z$av[18])/(sqrt(2*log(2)/pi))
6.35727e-06 #16bnw_500hz
#COMPARISON WITH OTHERS
5.591609e-06 #(5 hours test)
6.71548e-06 #STIM300 rad/s which is 1.3 deg/hr
1.459917e-05 #iMAR rad/s
0.0001488385 #XSens rad/s



