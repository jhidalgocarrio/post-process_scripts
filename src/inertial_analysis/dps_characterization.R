###############################################
# Post Processing R Script to compute         
# the Allan variance over the Pressure sensor
# Javier Hidalgo Carrió
# DFKI-RIC September 2011                            
###############################################

#libraries
library(allanvar)

# You may change this path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110915_dps_long_term/1_surface/")

##########################
# DPS TEST on Surface
##########################

load ("dps.0.log-dps.depth_samples.RData")
ls()


time <- as.data.frame(time)
names(time) <- seq(1, length(time))

position <- as.data.frame(position)
names(position) <- seq(1, length(position))

#Data time adaptation
sec <- as.integer(sapply(time,function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t1 <- as.numeric(paste(sec, usec, sep=''))

delta1 = NULL
for (i in 1:(length(t1)-1))
{
  delta1[i] <- (t1[i+1] - t1[i])/1000000
}

depth <- NULL
depth <- unlist(position [3, ])

########################## Z Axis ##########################
depthSamples <- ts (data.frame(depth), delta=mean(delta1))

#### Plotting the Depth samples in a file ####
png(filename = "dps_samples.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot(depthSamples)
dev.off()

#Frequency
frequency (depthSamples)

#### Calculating the Allan Variance for the DPS on the surface ####
avdps1 <- avar (depthSamples@.Data, frequency (depthSamples))

#### Plotting the results ####
x11()
plot (avdps1$time,sqrt(avdps1$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m)")

legend(10, 1e-04, c("dps_Allanvar"),  fill = c("blue"))


plotav(avdps1)

#### Plotting the results in a file ####
png(filename = "dps_allanvar.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avdps1$time,sqrt(avdps1$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m)")

legend(10, 1e-04, c("dps_Allanvar"),  fill = c("blue"))
dev.off()




# You may change this path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110915_dps_long_term/2_middle_dps_97cm_deep/")

##########################
# DPS TEST on Middle
##########################

load ("dps.0.log-dps.depth_samples.RData")
ls()


time <- as.data.frame(time)
names(time) <- seq(1, length(time))

position <- as.data.frame(position)
names(position) <- seq(1, length(position))

#Data time adaptation
sec <- as.integer(sapply(time,function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t1 <- as.numeric(paste(sec, usec, sep=''))

delta1 = NULL
for (i in 1:(length(t1)-1))
{
  delta1[i] <- (t1[i+1] - t1[i])/1000000
}

depth <- NULL
depth <- unlist(position [3, ])

########################## Z Axis ##########################
depthSamples <- ts (data.frame(depth), delta=mean(delta1))

#### Plotting the Depth samples in a file ####
png(filename = "dps_samples.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot(depthSamples)
dev.off()

#Frequency
frequency (depthSamples)

#### Calculating the Allan Variance for the DPS on the middle ####
avdps2 <- avar (depthSamples@.Data, frequency (depthSamples))

#### Plotting the results ####
x11()
plot (avdps2$time,sqrt(avdps2$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m)")

legend(10, 1e-04, c("dps_Allanvar"),  fill = c("blue"))


plotav(avdps2)

#### Plotting the results in a file ####
png(filename = "dps_allanvar.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avdps2$time,sqrt(avdps2$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m)")

legend(10, 1e-04, c("dps_Allanvar"),  fill = c("blue"))
dev.off()



# You may change this path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110915_dps_long_term/3_bottom/")

##########################
# DPS TEST on Bottom
##########################

load ("dps.0.log-dps.depth_samples.RData")
ls()


time <- as.data.frame(time)
names(time) <- seq(1, length(time))

position <- as.data.frame(position)
names(position) <- seq(1, length(position))

#Data time adaptation
sec <- as.integer(sapply(time,function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t1 <- as.numeric(paste(sec, usec, sep=''))

delta1 = NULL
for (i in 1:(length(t1)-1))
{
  delta1[i] <- (t1[i+1] - t1[i])/1000000
}

depth <- NULL
depth <- unlist(position [3, ])

########################## Z Axis ##########################
depthSamples <- ts (data.frame(depth), delta=mean(delta1))

#### Plotting the Depth samples in a file ####
png(filename = "dps_samples.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot(depthSamples)
dev.off()

#Frequency
frequency (depthSamples)

#### Calculating the Allan Variance for the DPS in the bottom ####
avdps3 <- avar (depthSamples@.Data, frequency (depthSamples))

#### Plotting the results ####
x11()
plot (avdps3$time,sqrt(avdps3$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m)")

legend(10, 1e-04, c("dps_Allanvar"),  fill = c("blue"))


plotav(avdps3)

#### Plotting the results in a file ####
png(filename = "dps_allanvar.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avdps3$time,sqrt(avdps3$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m)")

legend(10, 1e-04, c("dps_Allanvar"),  fill = c("blue"))
dev.off()



# You may change this path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110915_dps_long_term/4_air/")

##########################
# DPS TEST on Air
##########################

load ("dps.0.log-dps.depth_samples.RData")
ls()


time <- as.data.frame(time)
names(time) <- seq(1, length(time))

position <- as.data.frame(position)
names(position) <- seq(1, length(position))

#Data time adaptation
sec <- as.integer(sapply(time,function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t1 <- as.numeric(paste(sec, usec, sep=''))

delta1 = NULL
for (i in 1:(length(t1)-1))
{
  delta1[i] <- (t1[i+1] - t1[i])/1000000
}

depth <- NULL
depth <- unlist(position [3, ])

########################## Z Axis ##########################
depthSamples <- ts (data.frame(depth), delta=mean(delta1))

#### Plotting the Depth samples in a file ####
png(filename = "dps_samples.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot(depthSamples)
dev.off()

#Frequency
frequency (depthSamples)

#### Calculating the Allan Variance for the DPS on the air ####
avdps4 <- avar (depthSamples@.Data, frequency (depthSamples))

#### Plotting the results ####
x11()
plot (avdps4$time,sqrt(avdps4$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m)")

legend(10, 1e-04, c("dps_Allanvar"),  fill = c("blue"))


plotav(avdps4)

#### Plotting the results in a file ####
png(filename = "dps_allanvar.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (avdps4$time,sqrt(avdps4$av),log= "xy", xaxt="n" , yaxt="n", type="l", col="blue", xlab="", ylab="")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m)")

legend(10, 1e-04, c("dps_Allanvar"),  fill = c("blue"))
dev.off()

#########################################
## Allan Variance - Noise coefficients ##
#########################################
##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 #sample 76
approx (x=c(avdps1$time[4], avdps1$time[5]), y= c(sqrt(avdps1$av[4]), sqrt(avdps1$av[5])), n=100)

rwcoeff1 <- unlist(approx (x=c(avdps1$time[4], avdps1$time[5]), y= c(sqrt(avdps1$av[4]), sqrt(avdps1$av[5])), n=100)[2])[76]

#Standard deviation at 14Hz
rwcoeff1/sqrt(1/14)


#Test 2 #sample 76
approx (x=c(avdps2$time[4], avdps2$time[5]), y= c(sqrt(avdps2$av[4]), sqrt(avdps2$av[5])), n=100)

rwcoeff2 <- unlist(approx (x=c(avdps2$time[4], avdps2$time[5]), y= c(sqrt(avdps2$av[4]), sqrt(avdps2$av[5])), n=100)[2])[76]

#Standard deviation at 14Hz
rwcoeff2/sqrt(1/14)


#Test 3 #sample 76
approx (x=c(avdps3$time[4], avdps3$time[5]), y= c(sqrt(avdps3$av[4]), sqrt(avdps3$av[5])), n=100)

rwcoeff3 <- unlist(approx (x=c(avdps3$time[4], avdps3$time[5]), y= c(sqrt(avdps3$av[4]), sqrt(avdps3$av[5])), n=100)[2])[76]

#Standard deviation at 14Hz
rwcoeff3/sqrt(1/14)

#Test 4 #sample 76
approx (x=c(avdps4$time[4], avdps4$time[5]), y= c(sqrt(avdps4$av[4]), sqrt(avdps4$av[5])), n=100)

rwcoeff4 <- unlist(approx (x=c(avdps4$time[4], avdps4$time[5]), y= c(sqrt(avdps4$av[4]), sqrt(avdps4$av[5])), n=100)[2])[76]

#Standard deviation at 14Hz
rwcoeff4/sqrt(1/14)

# A gyro (or an accelerometer) is a "smart" sensor: It has an internal processor that takes measurements at a very high rate. The sensor output is sampled at a much lower rate. The output is an aggregate of the noisy raw measurements: a random walk with random step sizes (i.e., Brownian motion). The sampled data are best characterized as Brownian noise rather than white noise. Brownian noise is characterized by an angular random walk (ARW) coefficient (
# 3.5∘/√hr).
# 
# So, how to convert the spec value to a value you can use to simulate sensor output? The sampled data can be treated as a white noise process, but with the variance depending on the sampling interval:
# σω=ARW/sqrt(Δt)
