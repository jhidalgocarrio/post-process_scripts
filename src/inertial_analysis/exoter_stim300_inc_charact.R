###############################################
# Post Processing R Script to compute
# the Allan variance over the STIM300 5g
# Javier Hidalgo Carrió
# DFKI-RIC ESTEC March 2014
###############################################

#libraries
library(allanvar)
library(timeSeries)

# You may change this path
setwd ("/home/javi/exoter/development/data/20140325_stim300_test")

#load("stim300_inc_16bnw_500hz_analysis.Rdata")
#load("stim300_inc_16bnw_250hz_analysis.Rdata")
#load("stim300_inc_262bnw_500hz_analysis.Rdata")

#############
# TEST Inc
#############
#values <- read.table ("stim300_inc_16bnw_500hz.data", sep=" ")
#values <- read.table ("stim300_inc_16bnw_250hz.data", sep=" ")
values <- read.table ("stim300_inc_33bnw_125hz.data", sep=" ")
#values <- read.table ("stim300_inc_262bnw_500hz.data", sep=" ")

names(values) = c("time", "incx", "incy", "incz")

str(values)

#Time is in microseconds (10^-6)
delta <-diff(values$time)
delta <- mean(delta)/1000000

########################## X Axis ##########################
stim300Inc <- ts(as.numeric(array(na.omit(values$incx))), deltat=mean(delta))

#Frequency
frequency (stim300Inc)

#### Calculating the Allan Variance for the inclinometers ####
avinc1x <- avar (stim300Inc@.Data, frequency (stim300Inc))

########################## Y Axis ##########################
stim300Inc <- ts(as.numeric(array(na.omit(values$incy))), deltat=mean(delta))

#Frequency
frequency (stim300Inc)

#### Calculating the Allan Variance for the inclinometers ####
avinc1y <- avar (stim300Inc@.Data, frequency (stim300Inc))

########################## Z Axis ##########################
stim300Inc <- ts(as.numeric(array(na.omit(values$incz))), deltat=mean(delta))

#Frequency
frequency (stim300Inc)

#### Calculating the Allan Variance for the inclinometers ####
avinc1z <- avar (stim300Inc@.Data, frequency (stim300Inc))


#### Ploting the results ####
plot (avinc1x$time,sqrt(avinc1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avinc1y$time,sqrt(avinc1y$av), col="green")
lines (avinc1z$time,sqrt(avinc1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("InclinometerX", "InclinometerY", "InclinometerZ"),  fill = c("blue", "green", "red"))

#### Plotting the results in a file ####
png(filename = "stim300_inc_allanvar.png", width=2048, height=1536, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avinc1x$time,sqrt(avinc1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avinc1y$time,sqrt(avinc1y$av), col="green")
lines (avinc1z$time,sqrt(avinc1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=2.0, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("InclinometerX", "InclinometerY", "InclinometerZ"),  fill = c("blue", "green", "red"))

dev.off()

### Delete all values
rm (values, stim300Inc)

#### Save the result in a R image
#save.image (file = "stim300_inc_16bnw_500hz_analysis.Rdata")
#save.image (file = "stim300_inc_16bnw_250hz_analysis.Rdata")
#save.image (file = "stim300_inc_262bnw_500hz_analysis.Rdata")

#######################################
#### Calculate the values (STIM300 Inc) #
#######################################
plotav (avinc1x)

##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 #
approx (x=c(avinc1x$time[9], avinc1x$time[10]), y= c(sqrt(avinc1x$av[9]), sqrt(avinc1x$av[10])), n=100)
0.001406582 #16bnw_500hz
approx (x=c(avinc1x$time[8], avinc1x$time[9]), y= c(sqrt(avinc1x$av[8]), sqrt(avinc1x$av[9])), n=100)
0.001420615 #16bnw_250hz
approx (x=c(avinc1x$time[7], avinc1x$time[8]), y= c(sqrt(avinc1x$av[7]), sqrt(avinc1x$av[8])), n=100)
0.001427436 #33bnw_125hz
#COMPARISON WITH OTHERS

approx (x=c(avinc1y$time[9], avinc1y$time[10]), y= c(sqrt(avinc1y$av[9]), sqrt(avinc1y$av[10])), n=100)
0.001469949 #16bnw_500hz
approx (x=c(avinc1y$time[8], avinc1y$time[9]), y= c(sqrt(avinc1y$av[8]), sqrt(avinc1y$av[9])), n=100)
0.001471053 #16bnw_250hz
approx (x=c(avinc1y$time[7], avinc1y$time[8]), y= c(sqrt(avinc1y$av[7]), sqrt(avinc1y$av[8])), n=100)
0.001515822 #33bnw_125hz
#COMPARISON WITH OTHERS

approx (x=c(avinc1z$time[9], avinc1z$time[10]), y= c(sqrt(avinc1z$av[9]), sqrt(avinc1z$av[10])), n=100)
0.002011827 #16bnw_500hz
approx (x=c(avinc1z$time[8], avinc1z$time[9]), y= c(sqrt(avinc1z$av[8]), sqrt(avinc1z$av[9])), n=100)
0.002019287 #16bnw_250hz
approx (x=c(avinc1z$time[7], avinc1z$time[8]), y= c(sqrt(avinc1z$av[7]), sqrt(avinc1z$av[8])), n=100)
0.001966357 #33bnw_125hz
#COMPARISON WITH OTHERS

##
#Bias Instability, can be directly obtained by reading the 0 slope line 
##

#Test 1 Bias instability Coeff
sqrt(avinc1x$av[14])/(sqrt(2*log(2)/pi))
0.001083798
sqrt(avinc1x$av[13])/(sqrt(2*log(2)/pi))
0.001292219
#COMPARISON WITH OTHERS

sqrt(avinc1y$av[14])/(sqrt(2*log(2)/pi))
0.0009918666
sqrt(avinc1y$av[13])/(sqrt(2*log(2)/pi))
0.001160451
#COMPARISON WITH OTHERS

sqrt(avinc1z$av[14])/(sqrt(2*log(2)/pi))
0.001434095
sqrt(avinc1z$av[13])/(sqrt(2*log(2)/pi))
0.00146485
#COMPARISON WITH OTHERS

##
#Correlation time of the Bias Instability
##
avinc1x$time[13]
32.7676

avinc1y$time[13]
32.7676

avinc1z$time[13]
32.7676


##
#Rate Random Walk can be obtained by reading the allan variance value
#by a slope at +1/2. K=sqrt(3)*allandeviation(t)/sqrt(t)
# or K=sqrt((3*allandeviation(t))/sqrt)t))
##
sqrt((3.0 * avinc1x$av[18])/avinc1x$time[18])
4.1328e-05

sqrt((3.0 * avinc1y$av[18])/avinc1y$time[18])
3.351992e-05

sqrt((3.0 * avinc1z$av[18])/avinc1z$time[18])
6.312786e-05


