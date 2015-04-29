###############################################
# Post Processing R Script to compute         
# the Allan variance over the Xsens MTi           
# Javier Hidalgo Carrió
# DFKI-RIC May 2011                            
###############################################

#libraries
library(allanvar)

# You may change this path
#setwd("Z:/jhidalgo/AllanVariable/Data/TestCenter/Gyroscopes")
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110524_CG_JH_xsens_and_FOG_all_night/0_Sensor_Characterization")

#load("xsens_sensors.Rdata")

#############
# TEST Acc
#############
values <- read.table ("xsens_acc.csv", sep=" ")

names(values) = c("time", "accx", "accy", "accz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000

########################## X Axis ##########################
xsensAcc <- ts (data.frame(values$accx), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

#### Calculating the Allan Variance for the accelerometers ####
avacc1x <- avar (xsensAcc@.Data, frequency (xsensAcc))

########################## Y Axis ##########################
xsensAcc <- ts (data.frame(values$accy), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

#### Calculating the Allan Variance for the accelerometers ####
avacc1y <- avar (xsensAcc@.Data, frequency (xsensAcc))


########################## Z Axis ##########################
xsensAcc <- ts (data.frame(values$accz), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

#### Calculating the Allan Variance for the accelerometers ####
avacc1z <- avar (xsensAcc@.Data, frequency (xsensAcc))

#### Ploting Acc X
plot (xsensAcc)

### Delete all values
rm (values, xsensAcc)

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
png(filename = "xsens_acc.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avacc1x$time,sqrt(avacc1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avacc1y$time,sqrt(avacc1y$av), col="green")
lines (avacc1z$time,sqrt(avacc1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("AccelerometerX", "AccelerometerY", "AccelerometerZ"),  fill = c("blue", "green", "red"))

dev.off()


#############
# TEST Gyro
#############
values <- read.table ("xsens_gyro.csv", sep=" ")

names(values) = c("time", "gyrox", "gyroy", "gyroz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000


########################## X Axis ##########################
xsensGyro <- ts (data.frame(values$gyrox), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1x <- avar (xsensGyro@.Data, frequency (xsensGyro))

########################## Y Axis ##########################
xsensGyro <- ts (data.frame(values$gyroy), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1y <- avar (xsensGyro@.Data, frequency (xsensGyro))


########################## Z Axis ##########################
xsensGyro <- ts (data.frame(values$gyroz), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1z <- avar (xsensGyro@.Data, frequency (xsensGyro))

#### Plotting the results ####
x11()
plot (avgyro1x$time,sqrt(avgyro1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avgyro1y$time,sqrt(avgyro1y$av), col="green", lwd=1)
lines (avgyro1z$time,sqrt(avgyro1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 5e-03, c("GyroscopeX", "GyroscopeY", "GyroscopeZ"),  fill = c("blue", "green", "red"))

#dev.print(png, file="xsens_gyros.png", width=1024, height=768, bg = "white") # To save the x11 device in a png

#### Plotting the results in a file ####
png(filename = "xsens_gyros.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avgyro1x$time,sqrt(avgyro1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avgyro1y$time,sqrt(avgyro1y$av), col="green", lwd=1)
lines (avgyro1z$time,sqrt(avgyro1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 5e-03, c("GyroscopeX", "GyroscopeY", "GyroscopeZ"),  fill = c("blue", "green", "red"))
dev.off()

### Delete all values
rm (values, xsensGyro)

#### Save the result in a R image
#save.image (file = "xsens_sensors.Rdata")

##################
# Filtering
# Low-pass at 50Hz
########################## X Axis ##########################
Tc = 1.0/(2.0*pi*50.00)

filtervalues=NULL
filtervalues$gyrox[1] = values$gyrox[1];
for (i in 2: length(values$gyrox))
{
  filtervalues$gyrox[i]=filtervalues$gyrox[i-1]+(delta/Tc)*(values$gyrox[i]-filtervalues$gyrox[i-1])
}

filtervalues$time = values$time

#Allan Variance of filtered values
xsensGyro <- ts (data.frame(filtervalues$gyrox), start=c(filtervalues$time[1]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1xf <- avar (xsensGyro@.Data, frequency (xsensGyro))


#############
# TEST MAGN
#############
values <- read.table ("xsens_magn.csv", sep=" ")

names(values) = c("time", "magx", "magy", "magz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000


########################## X Axis ##########################
xsensMag <- ts (data.frame(values$magx), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensMag)

#### Calculating the Allan Variance for the gyroscope ####
avmag1x <- avar (xsensMag@.Data, frequency (xsensMag))

########################## Y Axis ##########################
xsensMag <- ts (data.frame(values$magy), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensMag)

#### Calculating the Allan Variance for the gyroscope ####
avmag1y <- avar (xsensMag@.Data, frequency (xsensMag))


########################## Z Axis ##########################
xsensMag <- ts (data.frame(values$magz), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensMag)

#### Calculating the Allan Variance for the gyroscope ####
avmag1z <- avar (xsensMag@.Data, frequency (xsensMag))


#### Plotting the results ####
x11()
plot (avmag1x$time,sqrt(avmag1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avmag1y$time,sqrt(avmag1y$av), col="green", lwd=1)
lines (avmag1z$time,sqrt(avmag1z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation ()")

legend(10, 5e-03, c("MagnetometerX", "MagnetometerY", "MagnetometerZ"),  fill = c("blue", "green", "red"))


####################################################################################################################################

# You may change this path
#setwd("Z:/jhidalgo/AllanVariable/Data/TestCenter/Gyroscopes")
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110518_CG_JH_xsens_and_FOG_all_night/0_Xsens_characterisation")

#############
# TEST Acc
#############
values <- read.table ("xsens_acc.csv", sep=" ")

names(values) = c("time", "accx", "accy", "accz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000

########################## X Axis ##########################
xsensAcc <- ts (data.frame(values$accx), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

#### Calculating the Allan Variance for the accelerometers ####
avacc2x <- avar (xsensAcc@.Data, frequency (xsensAcc))

########################## Y Axis ##########################
xsensAcc <- ts (data.frame(values$accy), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

#### Calculating the Allan Variance for the accelerometers ####
avacc2y <- avar (xsensAcc@.Data, frequency (xsensAcc))


########################## Z Axis ##########################
xsensAcc <- ts (data.frame(values$accz), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

#### Calculating the Allan Variance for the accelerometers ####
avacc2z <- avar (xsensAcc@.Data, frequency (xsensAcc))

#### Ploting Acc X
plot (xsensAcc)

### Delete all values
rm (values, xsensAcc)

#### Ploting the results ####
plot (avacc2x$time,sqrt(avacc2x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avacc2y$time,sqrt(avacc2y$av), col="green")
lines (avacc2z$time,sqrt(avacc2z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("AccelerometerX", "AccelerometerY", "AccelerometerZ"),  fill = c("blue", "green", "red"))

#### Plotting the results in a file ####
png(filename = "avxsens_acc.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avacc2x$time,sqrt(avacc2x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avacc2y$time,sqrt(avacc2y$av), col="green")
lines (avacc2z$time,sqrt(avacc2z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("AccelerometerX", "AccelerometerY", "AccelerometerZ"),  fill = c("blue", "green", "red"))

dev.off()


#############
# TEST Gyro
#############
values <- read.table ("xsens_gyro.csv", sep=" ")

names(values) = c("time", "gyrox", "gyroy", "gyroz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000

########################## X Axis ##########################
xsensGyro <- ts (data.frame(values$gyrox), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro2x <- avar (xsensGyro@.Data, frequency (xsensGyro))

########################## Y Axis ##########################
xsensGyro <- ts (data.frame(values$gyroy), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro2y <- avar (xsensGyro@.Data, frequency (xsensGyro))


########################## Z Axis ##########################
xsensGyro <- ts (data.frame(values$gyroz), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro2z <- avar (xsensGyro@.Data, frequency (xsensGyro))

#### Plotting the results ####
x11()
plot (avgyro2x$time,sqrt(avgyro2x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avgyro2y$time,sqrt(avgyro2y$av), col="green", lwd=1)
lines (avgyro2z$time,sqrt(avgyro2z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 5e-03, c("GyroscopeX", "GyroscopeY", "GyroscopeZ"),  fill = c("blue", "green", "red"))

#dev.print(png, file="xsens_gyros.png", width=1024, height=768, bg = "white") # To save the x11 device in a png

#### Plotting the results in a file ####
png(filename = "avxsens_gyros.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avgyro2x$time,sqrt(avgyro2x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avgyro2y$time,sqrt(avgyro2y$av), col="green", lwd=1)
lines (avgyro2z$time,sqrt(avgyro2z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 5e-03, c("GyroscopeX", "GyroscopeY", "GyroscopeZ"),  fill = c("blue", "green", "red"))
dev.off()

### Delete all values
rm (values, xsensGyro)


#############
# TEST MAGN
#############
values <- read.table ("xsens_magn.csv", sep=" ")

names(values) = c("time", "magx", "magy", "magz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000


########################## X Axis ##########################
xsensMag <- ts (data.frame(values$magx), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensMag)

#### Calculating the Allan Variance for the gyroscope ####
avmag2x <- avar (xsensMag@.Data, frequency (xsensMag))

########################## Y Axis ##########################
xsensMag <- ts (data.frame(values$magy), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensMag)

#### Calculating the Allan Variance for the gyroscope ####
avmag2y <- avar (xsensMag@.Data, frequency (xsensMag))


########################## Z Axis ##########################
xsensMag <- ts (data.frame(values$magz), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensMag)

#### Calculating the Allan Variance for the gyroscope ####
avmag2z <- avar (xsensMag@.Data, frequency (xsensMag))


#### Plotting the results ####
x11()
plot (avmag2x$time,sqrt(avmag2x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avmag2y$time,sqrt(avmag2y$av), col="green", lwd=1)
lines (avmag2z$time,sqrt(avmag2z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation ()")

legend(10, 5e-03, c("MagnetometerX", "MagnetometerY", "MagnetometerZ"),  fill = c("blue", "green", "red"))

### Delete all values
rm (values, xsensMag)

####################################################################################################################################
#Change the path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110527_CG_JH_xsens_and_FOG+temp_all_night/")
#load("xsens_sensors.Rdata")

#### TEMP ####
values <- read.table ("temp.data", sep="\t")

names(values) = c("time", "temp")

delta <- NULL
delta[1] = (values$time[20]- values$time[19])/1000000
delta[2] = (values$time[2000]- values$time[1999])/1000000
delta[3] = (values$time[1000]- values$time[999])/1000000

unclass(Sys.time())
as.POSIXct(values$time[1], origin="1970-01-01")
as.POSIXct(max(values$time[length(values$time)]), origin="1970-01-01")

tempSensor <- ts (data.frame(values$temp), delta=mean(delta))
plot(tempSensor, type="l", col="blue")
grid(lwd=1, col="orange")

#############
# TEST Acc
#############
values <- read.table ("xsens_acc.csv", sep=" ")

names(values) = c("time", "accx", "accy", "accz")

str(values)

#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000

########################## X Axis ##########################
xsensAcc <- ts (data.frame(values$accx[1:4270113]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

#### Calculating the Allan Variance for the accelerometers ####
avacc3x <- avar (xsensAcc@.Data, frequency (xsensAcc))

########################## Y Axis ##########################
xsensAcc <- ts (data.frame(values$accy[1:4270113]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

#### Calculating the Allan Variance for the accelerometers ####
avacc3y <- avar (xsensAcc@.Data, frequency (xsensAcc))


########################## Z Axis ##########################
xsensAcc <- ts (data.frame(values$accz[1:4270113]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

#### Calculating the Allan Variance for the accelerometers ####
avacc3z <- avar (xsensAcc@.Data, frequency (xsensAcc))

#### Ploting Acc X ####
x11()
png(filename = "acc_samples.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (xsensAcc)
dev.off()

### Delete all values
rm (values, xsensAcc)

#### Ploting the results ####
x11()
plotav (avacc3x)
x11()
plot (avacc3x$time,sqrt(avacc3x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avacc3y$time,sqrt(avacc3y$av), col="green")
lines (avacc3z$time,sqrt(avacc3z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("AccelerometerX", "AccelerometerY", "AccelerometerZ"),  fill = c("blue", "green", "red"))

#### Plotting the results in a file ####
png(filename = "avxsens_acc.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avacc3x$time,sqrt(avacc3x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avacc3y$time,sqrt(avacc3y$av), col="green")
lines (avacc3z$time,sqrt(avacc3z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("AccelerometerX", "AccelerometerY", "AccelerometerZ"),  fill = c("blue", "green", "red"))

dev.off()

#### Ploting only 2 hours of Acc X ####
xsensAcc <- ts (data.frame(values$accx[1:662400]), delta=mean(delta))

#Frequency
frequency (xsensAcc)

x11()
png(filename = "acc_samples_2h.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (xsensAcc)
dev.off()

#### Acc Temperature correlation ####
length(tempSensor@.Data)
length(xsensAcc@.Data)

## Interpolate with Akima ##
library (akima)
z <- aspline (tempSensor, n=length(xsensAcc@.Data), method="original")
png(filename = "accxvstemp.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (z$y, xsensAcc@.Data, col="blue", type="p")
dev.off()

## Linear regression model for Acc X ##
xsensAcc <- ts (data.frame(values$accx[1:662400]), delta=mean(delta))
lmtemp <- lm (xsensAcc@.Data ~ z$y)
png(filename = "accxvstemplm.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (z$y, xsensAcc@.Data, col="black", type="p", xlab="", ylab="")
title(main = "Xsens MTi Accelerometer Residual Thermal Drift", ylab = "Acceleration (m/s^2)", xlab = "Temperature (Celsius deg)")
legend(38, 0.1, c("Acc X", "Linear Regression"),  fill = c("black", "red"))
abline (lmtemp, col="red")
dev.off()

## Linear regression model for Acc Y ##
xsensAcc <- ts (data.frame(values$accy[1:662400]), delta=mean(delta))
lmtemp <- lm (xsensAcc@.Data ~ z$y)
png(filename = "accyvstemplm.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (z$y, xsensAcc@.Data, col="black", type="p", xlab="", ylab="")
title(main = "Xsens MTi Accelerometer Residual Thermal Drift", ylab = "Acceleration (m/s^2)", xlab = "Temperature (Celsius deg)")
legend(38, 0.22, c("Acc Y", "Linear Regression"),  fill = c("black", "red"))
abline (lmtemp, col="red")
dev.off()

## Linear regression model for Acc Z ##
xsensAcc <- ts (data.frame(values$accz[1:662400]), delta=mean(delta))
lmtemp <- lm (xsensAcc@.Data ~ z$y)
png(filename = "acczvstemplm.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (z$y, xsensAcc@.Data, col="black", type="p",  xlab="", ylab="")
title(main = "Xsens MTi Accelerometer Residual Thermal Drift", ylab = "Acceleration (m/s^2)", xlab = "Temperature (Celsius deg)")
legend(30, 9.93, c("Acc Z", "Linear Regression"),  fill = c("black", "red"))
abline (lmtemp, col="red")
dev.off()


#### End of Acc Temperature correlation ####

#############
# TEST Gyro
#############
values <- read.table ("xsens_gyro.csv", sep=" ")

names(values) = c("time", "gyrox", "gyroy", "gyroz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000

########################## X Axis ##########################
xsensGyro <- ts (data.frame(values$gyrox[1:4270113]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro3x <- avar (xsensGyro@.Data, frequency (xsensGyro))

########################## Y Axis ##########################
xsensGyro <- ts (data.frame(values$gyroy[1:4270113]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro3y <- avar (xsensGyro@.Data, frequency (xsensGyro))


########################## Z Axis ##########################
xsensGyro <- ts (data.frame(values$gyroz[1:4270113]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

#### Calculating the Allan Variance for the gyroscope ####
avgyro3z <- avar (xsensGyro@.Data, frequency (xsensGyro))

#### Ploting the gyros sample ####
x11()
png(filename = "gyro_samples.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (xsensGyro)
dev.off()

#### Ploting only 2 hours of Gyros X ####
xsensGyro <- ts (data.frame(values$gyrox[1:662400]), delta=mean(delta))

#Frequency
frequency (xsensGyro)

x11()
png(filename = "gyro_samples_2h.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (xsensGyro)
dev.off()
#### There is not Gyroscopes-Temperature correlation #### XSens MTi gyros are quite stable ####


#### Plotting the results ####
x11()
plotav(avgyro3x)
x11()
plot (avgyro3x$time,sqrt(avgyro3x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avgyro3y$time,sqrt(avgyro3y$av), col="green", lwd=1)
lines (avgyro3z$time,sqrt(avgyro3z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 5e-03, c("GyroscopeX", "GyroscopeY", "GyroscopeZ"),  fill = c("blue", "green", "red"))

#dev.print(png, file="xsens_gyros.png", width=1024, height=768, bg = "white") # To save the x11 device in a png

#### Plotting the results in a file ####
png(filename = "avxsens_gyros.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avgyro3x$time,sqrt(avgyro3x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avgyro3y$time,sqrt(avgyro3y$av), col="green", lwd=1)
lines (avgyro3z$time,sqrt(avgyro3z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 5e-03, c("GyroscopeX", "GyroscopeY", "GyroscopeZ"),  fill = c("blue", "green", "red"))
dev.off()

### Delete all values
rm (values, xsensGyro)
#############
# TEST MAGN
#############
values <- read.table ("xsens_magn.csv", sep=" ")

names(values) = c("time", "magx", "magy", "magz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000


########################## X Axis ##########################
xsensMag <- ts (data.frame(values$magx), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensMag)

#### Calculating the Allan Variance for the gyroscope ####
avmag3x <- avar (xsensMag@.Data, frequency (xsensMag))

########################## Y Axis ##########################
xsensMag <- ts (data.frame(values$magy), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensMag)

#### Calculating the Allan Variance for the gyroscope ####
avmag3y <- avar (xsensMag@.Data, frequency (xsensMag))


########################## Z Axis ##########################
xsensMag <- ts (data.frame(values$magz), start=c(values$time[1]), delta=mean(delta))

#Frequency
frequency (xsensMag)

#### Calculating the Allan Variance for the gyroscope ####
avmag3z <- avar (xsensMag@.Data, frequency (xsensMag))


#### Plotting the results ####
x11()
plot (avmag3x$time,sqrt(avmag3x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avmag3y$time,sqrt(avmag3y$av), col="green", lwd=1)
lines (avmag3z$time,sqrt(avmag3z$av), col="red")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation ()")

legend(10, 5e-03, c("MagnetometerX", "MagnetometerY", "MagnetometerZ"),  fill = c("blue", "green", "red"))

### Delete all values
rm (values, xsensMag)

#### Save the result in a R image
save.image (file = "xsens_sensors.Rdata")


################################
#### Comparison of Analysis ####
################################
x11()#png(filename = "avxsens_acc_comparison.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avacc1x$time,sqrt(avacc1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avacc1y$time,sqrt(avacc1y$av), col="green")
lines (avacc1z$time,sqrt(avacc1z$av), col="red")
lines (avacc2x$time,sqrt(avacc2x$av), col="blue", type='o')
lines (avacc2y$time,sqrt(avacc2y$av), col="green", type='o')
lines (avacc2z$time,sqrt(avacc2z$av), col="red", type='o')
lines (avacc3x$time,sqrt(avacc3x$av), col="blue", type='c')
lines (avacc3y$time,sqrt(avacc3y$av), col="green", type='c')
lines (avacc3z$time,sqrt(avacc3z$av), col="red", type='c')
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 1e-02, c("AccelerometerX", "AccelerometerY", "AccelerometerZ"),  fill = c("blue", "green", "red"))
#dev.off()



x11()#png(filename = "avxsens_gyros_comparison.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avgyro1x$time,sqrt(avgyro1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
lines (avgyro1y$time,sqrt(avgyro1y$av), col="green")
lines (avgyro1z$time,sqrt(avgyro1z$av), col="red")
lines (avgyro2x$time,sqrt(avgyro2x$av), col="blue", type='o')
lines (avgyro2y$time,sqrt(avgyro2y$av), col="green", type='o')
lines (avgyro2z$time,sqrt(avgyro2z$av), col="red", type='o')
lines (avgyro3x$time,sqrt(avgyro3x$av), col="blue", type='c')
lines (avgyro3y$time,sqrt(avgyro3y$av), col="green", type='c')
lines (avgyro3z$time,sqrt(avgyro3z$av), col="red", type='c')
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(10, 5e-03, c("GyroscopeX", "GyroscopeY", "GyroscopeZ"),  fill = c("blue", "green", "red"))
#dev.off()

#######################################
#### Calculate the values (Xsens Acc) #
#######################################
plotav (avacc2x)

##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 # sample 57
approx (x=c(avacc1x$time[7], avacc1x$time[8]), y= c(sqrt(avacc1x$av[7]), sqrt(avacc1x$av[8])), n=100)

0.0010122614 #m/s^2/sqrt(Hz)

approx (x=c(avacc1y$time[7], avacc1y$time[8]), y= c(sqrt(avacc1y$av[7]), sqrt(avacc1y$av[8])), n=100)

0.0010506054 #m/s^2/sqrt(Hz)

approx (x=c(avacc1z$time[7], avacc1z$time[8]), y= c(sqrt(avacc1z$av[7]), sqrt(avacc1z$av[8])), n=100)

0.001088147 #m/s^2/sqrt(Hz)

#Test 2
approx (x=c(avacc2x$time[7], avacc2x$time[8]), y= c(sqrt(avacc2x$av[7]), sqrt(avacc2x$av[8])), n=100)

0.0011017605 #m/s^2/sqrt(Hz)

approx (x=c(avacc2y$time[7], avacc2y$time[8]), y= c(sqrt(avacc2y$av[7]), sqrt(avacc2y$av[8])), n=100)

0.0011035893 #m/s^2/sqrt(Hz)

approx (x=c(avacc2z$time[7], avacc2z$time[8]), y= c(sqrt(avacc2z$av[7]), sqrt(avacc2z$av[8])), n=100)

0.001167750 #m/s^2/sqrt(Hz)


#Test 3
approx (x=c(avacc3x$time[7], avacc3x$time[8]), y= c(sqrt(avacc3x$av[7]), sqrt(avacc3x$av[8])), n=100)

0.0010310305 #m/s^2/sqrt(Hz)

approx (x=c(avacc3y$time[7], avacc3y$time[8]), y= c(sqrt(avacc3y$av[7]), sqrt(avacc3y$av[8])), n=100)

0.001318912 #m/s^2/sqrt(Hz)

approx (x=c(avacc3z$time[7], avacc3z$time[8]), y= c(sqrt(avacc3z$av[7]), sqrt(avacc3z$av[8])), n=100)

0.0011245297 #m/s^2/sqrt(Hz)


##
#Bias Instability, can be directly obtained by reading the 0 slope line 
##
#Test 1 Bias instability Coeff
sqrt(avacc1x$av[16])/(sqrt(2*log(2)/pi))
0.0004976398 #m*s^2

sqrt(avacc1y$av[16])/(sqrt(2*log(2)/pi))
0.0005772482

sqrt(avacc1z$av[16])/(sqrt(2*log(2)/pi))
0.0006744514


#Test 2 Bias instability Coeff
sqrt(avacc2x$av[16])/(sqrt(2*log(2)/pi))
0.0006231288

sqrt(avacc2y$av[16])/(sqrt(2*log(2)/pi))
0.0005759302

sqrt(avacc2z$av[16])/(sqrt(2*log(2)/pi))
0.0004979319

#Test 3 Bias instability Coeff
sqrt(avacc2x$av[13])/(sqrt(2*log(2)/pi))
0.0008291801

sqrt(avacc2y$av[13])/(sqrt(2*log(2)/pi))
0.001136927

sqrt(avacc2z$av[13])/(sqrt(2*log(2)/pi))
0.0009820723

#######################################
#### Calculate the values (Xsens Gyro)#
#######################################
plotav (avgyro2x)

##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 # sample 57
approx (x=c(avgyro1x$time[7], avgyro1x$time[8]), y= c(sqrt(avgyro1x$av[7]), sqrt(avgyro1x$av[8])), n=100)

0.0006643596 #rad/s/sqrt(Hz)

approx (x=c(avgyro1y$time[7], avgyro1y$time[8]), y= c(sqrt(avgyro1y$av[7]), sqrt(avgyro1y$av[8])), n=100)

0.0006978859 #rad/s/sqrt(Hz)

approx (x=c(avgyro1z$time[7], avgyro1z$time[8]), y= c(sqrt(avgyro1z$av[7]), sqrt(avgyro1z$av[8])), n=100)

0.0006390777 #rad/s/sqrt(Hz)

#Test 2
approx (x=c(avgyro2x$time[7], avgyro2x$time[8]), y= c(sqrt(avgyro2x$av[7]), sqrt(avgyro2x$av[8])), n=100)

0.0006562637 #rad/s/sqrt(Hz)

approx (x=c(avgyro2y$time[7], avgyro2y$time[8]), y= c(sqrt(avgyro2y$av[7]), sqrt(avgyro2y$av[8])), n=100)

0.0006890892 #rad/s/sqrt(Hz)

approx (x=c(avgyro2z$time[7], avgyro2z$time[8]), y= c(sqrt(avgyro2z$av[8]), sqrt(avgyro2z$av[8])), n=100)

0.0005363366 #rad/s/sqrt(Hz)

#Test 3
approx (x=c(avgyro3x$time[7], avgyro3x$time[8]), y= c(sqrt(avgyro3x$av[7]), sqrt(avgyro3x$av[8])), n=100)

0.0006898864 #rad/s/sqrt(Hz)

approx (x=c(avgyro3y$time[7], avgyro3y$time[8]), y= c(sqrt(avgyro3y$av[7]), sqrt(avgyro3y$av[8])), n=100)

0.0007219069 #rad/s/sqrt(Hz)

approx (x=c(avgyro3z$time[7], avgyro3z$time[8]), y= c(sqrt(avgyro3z$av[8]), sqrt(avgyro3z$av[8])), n=100)

0.0005708627 #rad/s/sqrt(Hz)


##
#Bias Instability, can be directly obtained by reading the 0 slope line 
##

#Test 1 Bias instability Coeff
sqrt(avgyro1x$av[15])/(sqrt(2*log(2)/pi))
0.0001463489 #rad/s

sqrt(avgyro1y$av[15])/(sqrt(2*log(2)/pi))
0.000151518

sqrt(avgyro1z$av[15])/(sqrt(2*log(2)/pi))
0.0001488385


#Test 2 Bias instability Coeff
sqrt(avgyro2x$av[15])/(sqrt(2*log(2)/pi))
0.000146352 #rad/s

sqrt(avgyro2y$av[15])/(sqrt(2*log(2)/pi))
0.0001526533

sqrt(avgyro2z$av[15])/(sqrt(2*log(2)/pi))
0.0001721282


#Test 3 Bias instability Coeff
sqrt(avgyro3x$av[15])/(sqrt(2*log(2)/pi))
0.0001628367 #rad/s

sqrt(avgyro3y$av[15])/(sqrt(2*log(2)/pi))
0.0001889337

sqrt(avgyro3z$av[15])/(sqrt(2*log(2)/pi))
0.0001832527


#######################################
#### Calculate the values (Xsens Mag)#
#######################################
plotav (avmag1x)


#Test 1 # sample 57
approx (x=c(avmag1x$time[7], avmag1x$time[8]), y= c(sqrt(avmag1x$av[7]), sqrt(avmag1x$av[8])), n=100)

0.0004290766

approx (x=c(avmag1y$time[7], avmag1y$time[8]), y= c(sqrt(avmag1y$av[7]), sqrt(avmag1y$av[8])), n=100)

9.207130e-05

approx (x=c(avmag1z$time[7], avmag1z$time[8]), y= c(sqrt(avmag1z$av[7]), sqrt(avmag1z$av[8])), n=100)

1.061307e-04

#Test 2
approx (x=c(avmag2x$time[7], avmag2x$time[8]), y= c(sqrt(avmag2x$av[7]), sqrt(avmag2x$av[8])), n=100)
0.0004013003

approx (x=c(avmag2y$time[7], avmag2y$time[8]), y= c(sqrt(avmag2y$av[7]), sqrt(avmag2y$av[8])), n=100)

8.659183e-05

approx (x=c(avmag2z$time[7], avmag2z$time[8]), y= c(sqrt(avmag2z$av[8]), sqrt(avmag2z$av[8])), n=100)
7.583042e-05


#Test 3
approx (x=c(avmag3x$time[7], avmag3x$time[8]), y= c(sqrt(avmag3x$av[7]), sqrt(avmag3x$av[8])), n=100)


approx (x=c(avmag3y$time[7], avmag3y$time[8]), y= c(sqrt(avmag3y$av[7]), sqrt(avmag3y$av[8])), n=100)


approx (x=c(avmag3z$time[7], avmag3z$time[8]), y= c(sqrt(avmag3z$av[8]), sqrt(avmag3z$av[8])), n=100)

