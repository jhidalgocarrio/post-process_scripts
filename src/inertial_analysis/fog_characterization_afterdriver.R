###############################################
# Post Processing R Script to compute         
# the Allan variance over the FOG KVH DSP3000            
# Javier Hidalgo Carrió
# DFKI-RIC May 2011                            
###############################################

#libraries
library(allanvar)

# You may change this path
setwd ("~/CUSLAM/experiments/20110829_JH_AllanVariance")

#############
# TEST FOG
#############
load ("dsp3000.28.log-dsp3000.rotation.RData")
ls()

##Only gyros are valid for FOG data
rm(acc,mag)


##Process the time and delta_time
sec <- as.integer(sapply(time,function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t1 <- as.numeric(paste(sec, usec, sep=''))

delta1 = NULL
for (i in 1:(length(t1)-1))
{
  delta1[i] <- (t1[i+1] - t1[i])/1000000
}

##Process the data
#time <- as.data.frame(time)
#names(time) <- seq(1, length(time))

gyro <- as.data.frame(gyro)
names(gyro) <- seq(1, length(gyro))

gyroz <- unlist(gyro [3, ])

########################## Z Axis ##########################
fogGyro <- ts (data.frame(gyroz), delta=mean(0.01))

#### Plotting the FOG samples in a file ####
png(filename = "fog_samples.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot(fogGyro, xlab="", ylab="")
title(main = "Fiber Optic Gyroscope - Output", xlab = "Times (sec)", ylab = "Angular Velocity (rad/s)")
dev.off()


#Frequency
frequency (fogGyro)

#### Calculating the Allan Variance for the accelerometers ####
avfog1x <- avar (fogGyro@.Data, frequency (fogGyro))

#### Plotting the results ####
x11()
png(filename = "av_fog.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avfog1x$time,sqrt(avfog1x$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")
legend(10, 1e-04, c("fog_zaxis"),  fill = c("blue"))
dev.off()


png(filename = "av_fog_errorbar.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plotav(avfog1x)
dev.off()

###################### Sensor Analysis #######################
plotav (avfog1x)
D2R = pi/180;
R2D = 180/pi

##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 #sample 49
approx (x=c(avfog1x$time[7], avfog1x$time[8]), y= c(sqrt(avfog1x$av[7]), sqrt(avfog1x$av[8])), n=100)

1.355538e-05 #rad/sqrt(Hz)

1.355538e-05 * R2D * 60 # from rad/sqrt(s) to deg/sqrt(h)

##
#Bias Instability, can be directly obtained by reading the 0 slope line 
##

#Bias instability Coefficient
sqrt(avfog1x$av[15])/(sqrt(2*log(2)/pi))
1.843885e-06 #rad/s

1.843885e-06 * R2D * 3600 # from rad/s to deg/h


save.image(file="dsp3000.28.log-dsp3000.rotation.RData")
