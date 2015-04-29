###############################################
# Post Processing R Script to compute
# the Allan variance over the FOG KVH DSP3000
# Javier Hidalgo Carrio
# DFKI-RIC May 2011
###############################################

#libraries
library(allanvar)

# You may change this path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110524_CG_JH_xsens_and_FOG_all_night/0_Sensor_Characterization")

#############
# TEST FOG
#############
values <- read.table ("fog_zaxis.csv", sep=" ")

names(values) = c("time", "gyroz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000

########################## Z Axis ##########################
fogGyro <- ts (data.frame(values$gyroz), delta=mean(delta))

#### Plotting the FOG samples in a file ####
png(filename = "fog_samples.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot(fogGyro)
dev.off()


#Frequency
frequency (fogGyro)

#### Calculating the Allan Variance for the accelerometers ####
avfog1x <- avar (fogGyro@.Data, frequency (fogGyro))

#### Plotting the results ####
x11()
plot (avfog1x$time,sqrt(avfog1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 1e-04, c("fog_zaxis"),  fill = c("blue"))


plotav(avfog1x)

#### Plotting the results in a file ####
png(filename = "fog_gyro.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avfog1x$time,sqrt(avfog1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 1e-04, c("fog_zaxis"),  fill = c("blue"))
dev.off()

####################################################################################################################################
#Change the path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110518_CG_JH_xsens_and_FOG_all_night/0_Gyro_characterisation")
#load (file = "fog_gyro.Rdata")

#############
# TEST FOG
#############
values <- read.table ("fog_orient.csv", sep="\t")

names(values) = c("time", "gyroz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000

########################## Z Axis ##########################
fogGyro <- ts (data.frame(values$gyroz), delta=mean(delta))

#### Plotting the FOG samples in a file ####
png(filename = "fog_isamples.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot(fogGyro)
dev.off()


#Frequency
frequency (fogGyro)

#### Calculating the Allan Variance for the accelerometers ####
avfog2x <- avari (fogGyro@.Data, frequency (fogGyro))

#### Plotting the results ####
x11()
plot (avfog1x$time,sqrt(avfog1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 1e-04, c("fog_zaxis"),  fill = c("blue"))


plotav(avfog2x)

#### Plotting the results in a file ####
png(filename = "fog_gyro.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avfog2x$time,sqrt(avfog2x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 1e-04, c("fog_zaxis"),  fill = c("blue"))
dev.off()

####################################################################################################################################
#Change the path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110527_CG_JH_xsens_and_FOG+temp_all_night/")

#############
# TEST FOG
#############

#### TEMP ####
values <- read.table ("temp.data", sep="\t")

names(values) = c("time", "temp")

delta[1] = (values$time[20]- values$time[19])/1000000
delta[2] = (values$time[2000]- values$time[1999])/1000000
delta[3] = (values$time[1000]- values$time[999])/1000000

tempSensor <- ts (data.frame(values$temp), delta=mean(delta))

#### Plotting the FOG samples in a file ####
png(filename = "temp_sensor.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot(tempSensor, type="l", col="blue")
grid(lwd=1, col="orange")
dev.off()


#### FOG ####
values <- read.table ("fog_zaxis.csv", sep=" ")

names(values) = c("time", "gyroz")

str(values)
#Time is in microseconds (10^-6)
delta[1] = (values$time[10]-values$time[9])/1000000
delta[2] = (values$time[100]-values$time[99])/1000000
delta[3] = (values$time[1000]-values$time[999])/1000000
delta[4] = (values$time[6]-values$time[5])/1000000

# Movementes at the end of the data record (because someone moved Dagon)
# Therefore, it taked out the last 2 hours (662400 samples)
########################## Z Axis ##########################
fogGyro <- ts (data.frame(values$gyroz[1:4270113]), delta=mean(delta))

#### Plotting the FOG samples in a file ####
png(filename = "fog_samples_ok.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot(fogGyro)
dev.off()

#Frequency
frequency (fogGyro)

#### Calculating the Allan Variance for the accelerometers ####
avfog3x <- avar (fogGyro@.Data, frequency (fogGyro))

#### Plotting the results ####
x11()
plot (avfog3x$time,sqrt(avfog3x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 1e-04, c("fog_zaxis"),  fill = c("blue"))


#### Plotting the results in a file ####
png(filename = "fog_gyro.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avfog3x$time,sqrt(avfog3x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 1e-04, c("fog_zaxis"),  fill = c("blue"))
dev.off()

png(filename = "fog_gyro+errorbar.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plotav(avfog3x)
dev.off()



#### Save the result in a R image
save.image (file = "fog_gyro.Rdata")


################################
#### Comparison of Analysis ####
################################
png(filename = "fog_gyro_comparison.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avfog1x$time,sqrt(avfog1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
#lines (avfog2x$time,sqrt(avfog2x$av), col="green", lwd=2)
lines (avfog3x$time,sqrt(avfog3x$av), col="red", lwd=2)
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 1e-04, c("Test1 (24-May)", "Test2(18-May)", "Test3(27-May)"),  fill = c("blue", "green", "red"))
legend(10, 1e-04, c("Test1 (24-May)", "Test3(27-May)"),  fill = c("blue", "red"))
dev.off()


#######################################
#### Calculate the values (Axis X) ####
#######################################
plotav (avfog2x)
D2R = pi/180;
R2D = 180/pi

##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 #sample 47
approx (x=c(avfog1x$time[7], avfog1x$time[8]), y= c(sqrt(avfog1x$av[7]), sqrt(avfog1x$av[8])), n=100)

1.302609e-05 #rad/sqrt(Hz)


#Test 2
approx (x=c(avfog2x$time[7], avfog2x$time[8]), y= c(sqrt(avfog2x$av[7]), sqrt(avfog2x$av[8])), n=100)

8.308074e-05 #rad/sqrt(Hz)

#Test 3
approx (x=c(avfog3x$time[7], avfog3x$time[8]), y= c(sqrt(avfog3x$av[7]), sqrt(avfog3x$av[8])), n=100)

1.319023e-05 #rad/sqrt(Hz)

1.319023e-05 * R2D * 3600

##
#Bias Instability, can be directly obtained by reading the 0 slope line 
##

#Bias instability Coefficient
sqrt(avfog1x$av[19])/(sqrt(2*log(2)/pi))
6.464593e-07 #rad/s

#Bias instability Coefficient
sqrt(avfog2x$av[18])/(sqrt(2*log(2)/pi))
9.6493e-07 #rad/s

#Bias instability Coefficient
sqrt(avfog3x$av[16])/(sqrt(2*log(2)/pi))
1.282000e-06 #rad/s

1.282000e-06 * R2D * 3600

##
#Rate Random Walk can be obtained by reading the allan variance value
#by a slope at +1/2. K=sqrt(3)*allandeviation(t)/sqrt(t)
##
sqrt(avfog3x$av[20])/(sqrt(avfog3x$time[20]/3))
1.902671e-08

