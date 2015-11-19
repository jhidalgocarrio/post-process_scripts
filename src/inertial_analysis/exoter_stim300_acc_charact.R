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
setwd ("/home/javi/exoter/development/data/20140325_stim300_test")

#load("stim300_acc_16bnw_500hz_analysis.Rdata")
#load("stim300_acc_16bnw_250hz_analysis.Rdata")
#load("stim300_acc_262bnw_500hz_analysis.Rdata")

#############
# TEST Acc
#############
#values <- read.table ("stim300_acc_16bnw_500hz.data", sep=" ")
values <- read.table ("stim300_acc_16bnw_250hz.data", sep=" ")
#values <- read.table ("stim300_acc_262bnw_500hz.data", sep=" ")
#values <- read.table ("stim300_acc_33bnw_125hz.data", sep=" ")

names(values) = c("time", "accx", "accy", "accz")

str(values)

#Time is in microseconds (10^-6)
delta <- diff(values$time)
delta <- mean(delta)/1000000

########################## X Axis ##########################
stim300Acc <- ts(as.numeric(array(na.omit(values$accx))), deltat=mean(delta))

#Frequency
frequency (stim300Acc)

#### Calculating the Allan Variance for the accelerometers ####
avacc1x <- avar (stim300Acc@.Data, frequency (stim300Acc))

########################## Y Axis ##########################
stim300Acc <- ts(as.numeric(array(na.omit(values$accy))), deltat=mean(delta))

#Frequency
frequency (stim300Acc)

#### Calculating the Allan Variance for the accelerometers ####
avacc1y <- avar (stim300Acc@.Data, frequency (stim300Acc))

########################## Z Axis ##########################
stim300Acc <- ts(as.numeric(array(na.omit(values$accz))), deltat=mean(delta))

#Frequency
frequency (stim300Acc)

#### Calculating the Allan Variance for the accelerometers ####
avacc1z <- avar (stim300Acc@.Data, frequency (stim300Acc))


#### Ploting the results ####
plot (avacc1x$time,sqrt(avacc1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avacc1y$time,sqrt(avacc1y$av), col="green")
lines (avacc1z$time,sqrt(avacc1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("AccelerometerX", "AccelerometerY", "AccelerometerZ"),  fill = c("blue", "green", "red"))

#### Plotting the results in a file ####
png(filename = "stim300_acc_allanvar.png", width=2048, height=1536, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avacc1x$time,sqrt(avacc1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avacc1y$time,sqrt(avacc1y$av), col="green")
lines (avacc1z$time,sqrt(avacc1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=2.0, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("AccelerometerX", "AccelerometerY", "AccelerometerZ"),  fill = c("blue", "green", "red"))

dev.off()

### Delete all values
rm (values, stim300Acc)

#### Save the result in a R image
#save.image (file = "stim300_acc_16bnw_500hz_analysis.Rdata")
#save.image (file = "stim300_acc_16bnw_250hz_analysis.Rdata")
#save.image (file = "stim300_acc_262bnw_500hz_analysis.Rdata")

#######################################
#### Calculate the values (STIM300 Acc) #
#######################################
plotav (avacc1x)

##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 # sample 96
approx (x=c(avacc1x$time[9], avacc1x$time[10]), y= c(sqrt(avacc1x$av[9]), sqrt(avacc1x$av[10])), n=100)
0.0005420144 #m/s^2/sqrt(Hz) or m/s/sqrt(s) 16bnw_500hz
0.0006062538 #m/s^2/sqrt(Hz) 262bnw_500hz
approx (x=c(avacc1x$time[8], avacc1x$time[9]), y= c(sqrt(avacc1x$av[8]), sqrt(avacc1x$av[9])), n=100)
0.0005583535 #m/s^2/sqrt(Hz) 16bnw_250hz
approx (x=c(avacc1x$time[7], avacc1x$time[8]), y= c(sqrt(avacc1x$av[7]), sqrt(avacc1x$av[8])), n=100)
0.0005303703 #m/s^2/sqrt(Hz) 33bnw_125hz

approx (x=c(avacc1y$time[9], avacc1y$time[10]), y= c(sqrt(avacc1y$av[9]), sqrt(avacc1y$av[10])), n=100)
0.0004919161 #m/s^2/sqrt(Hz) 16bnw_500hz
0.0004973285 #m/s^2/sqrt(Hz) 262bnw_500hz
approx (x=c(avacc1y$time[8], avacc1y$time[9]), y= c(sqrt(avacc1y$av[8]), sqrt(avacc1y$av[9])), n=100)
0.0005131682 #m/s^2/sqrt(Hz) 16bnw_250hz
approx (x=c(avacc1y$time[7], avacc1y$time[8]), y= c(sqrt(avacc1y$av[7]), sqrt(avacc1y$av[8])), n=100)
0.0004815498 #m/s^2/sqrt(Hz) 33bnw_125hz

approx (x=c(avacc1z$time[9], avacc1z$time[10]), y= c(sqrt(avacc1z$av[9]), sqrt(avacc1z$av[10])), n=100)
0.0004758711 #m/s^2/sqrt(Hz) 16bnw_500hz
approx (x=c(avacc1z$time[8], avacc1z$time[9]), y= c(sqrt(avacc1z$av[8]), sqrt(avacc1z$av[9])), n=100)
0.0004908665 #m/s^2/sqrt(Hz) 16bnw_250hz
approx (x=c(avacc1z$time[7], avacc1z$time[8]), y= c(sqrt(avacc1z$av[7]), sqrt(avacc1z$av[8])), n=100)
0.0004815498 #m/s^2/sqrt(Hz) 33bnw_125hz

#DATASHEET for ACC 5g
0.000353 #m/s^2/sqrt(Hz)

##
#Bias Instability, can be directly obtained by reading the 0 slope line 
##
#Test 1 Bias instability Coeff
sqrt(avacc1x$av[16])/(sqrt(2*log(2)/pi))
0.0003073389 # #16bnw_500hz(4 hours test 5g acc)
sqrt(avacc1x$av[15])/(sqrt(2*log(2)/pi))
0.0004368486 #16bnw_250hz

#COMPARISON WITH OTHERS
0.0005639343 # (5 hours test) 10g stim300 acc coincident with datasheet value
0.0003110737 #STIM300 m/s^2 datasheet 0.0005 m/s^2
0.0001213269 #IMAR m/s^2 datasheet 0.01 m/s^2
0.0004976398 #XSens m/s^2

sqrt(avacc1y$av[18])/(sqrt(2*log(2)/pi))
0.0002858801 #16bnw_500hz
sqrt(avacc1y$av[17])/(sqrt(2*log(2)/pi))
0.0003441604 #16bnw_250hz

#COMPARISON WITH OTHERS
0.0006412585 # (5 hours test) 10g stim300
0.0003967415 #STIM300 m/s^2
0.000117481 #iMAR m/s^2
0.0005772482 #XSens m/s^2

sqrt(avacc1z$av[17])/(sqrt(2*log(2)/pi))
0.0002847117 #16bnw_500hz
sqrt(avacc1z$av[16])/(sqrt(2*log(2)/pi))
0.0003097561 #16bnw_250hz

#COMPARISON WITH OTHERS
0.0005866994 # (5 hours test) 10g stim300 acc coincident with datasheet value
0.0003779767 #STIM300 m/s^2
0.0007197698 #iMAR m/s^2
0.0006744514 #XSens m/s^2

