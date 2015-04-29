###############################################
# Post Processing R Script to analyse         
# the orientation estimator using Xsens and FOG          
# Javier Hidalgo Carrió
# DFKI-RIC August 2011                            
###############################################

# You may change this path
setwd ("~/CUSLAM/experiments/20110829_JH_AllanVariance")

######################
# TEST FOG INTEGRATION
######################
load ("dsp3000.28.log-dsp3000.orientation_samples.RData")
ls()


time <- as.data.frame(time)
names(time) <- seq(1, length(time))

orientation <- as.data.frame(orientation)
names(orientation) <- seq(1, length(orientation))


#Data time adaptation
sec <- as.integer(sapply(time, function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t1 <- as.numeric(paste(sec, usec, sep=''))

delta1 = NULL
for (i in 1:(length(t1)-1))
{
  delta1[i] <- (t1[i+1] - t1[i])/1000000
}


attitude1 <- NULL
attitude1$roll <- unlist(orientation [1, ])
attitude1$pitch <- unlist(orientation [2, ])
attitude1$yaw <- unlist(orientation [3, ])

R2D = 180/pi
x11()
plot (attitude1$roll*R2D, type="l")

x11()
plot (attitude1$pitch*R2D, type="l")

x11()
plot (attitude1$yaw*R2D, type="l")


save.image(file="dsp3000.28.log-dsp3000.orientation_samples.RData")

# You may change this path
setwd ("~/CUSLAM/experiments/20110829_JH_AllanVariance")

#######################
# TEST ORIENTATION EST
#######################
load ("orientation_estimator.0.log-orientation_estimator.attitude_b_g.RData")
ls()


time <- as.data.frame(time)
names(time) <- seq(1, length(time))

orientation <- as.data.frame(orientation)
names(orientation) <- seq(1, length(orientation))

#Data time adaptation
sec <- as.integer(sapply(time, function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t2 <- as.numeric(paste(sec, usec, sep=''))

delta2 = NULL
for (i in 1:(length(t2)-1))
{
  delta2[i] <- (t2[i+1] - t2[i])/1000000
}


attitude2 <- NULL
attitude2$roll <- unlist(orientation [1, ])
attitude2$pitch <- unlist(orientation [2, ])
attitude2$yaw <- unlist(orientation [3, ])

R2D = 180/pi
x11()
plot (attitude2$roll*R2D, type="l")

x11()
plot (attitude2$pitch*R2D, type="l")

x11()
plot ((attitude2$yaw*R2D), type="l")

## Comparing Yaw FOG againts Yaw orientation_estimator
testfog <- ts(-attitude1$yaw*R2D, delta=mean(delta2))
testatt <- ts((attitude2$yaw*R2D)+38.18357, delta=mean(delta2))

png(filename = "static_heading.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (testfog, type="l", xaxt="n" , yaxt="n",xlab="", ylab="") #-38.18357 is the min angle
lines(testatt, type="l", col="blue")
axis(2, c(0, -2.5, -5, -7.5, -10, -12.5, -15))
axis(1, c(0, 1800, 3600, 5400, 7200, 9000))
grid (lwd=1, col="black")
title(main = "Estatic Dagon Heading Drift", xlab = "Time (sec)", ylab = "Angle (degrees)")
legend(4000, -1, c("fog incremental", "orientation estimator"),  fill = c("black", "blue"))
dev.off()

save.image(file="orientation_estimator.0.log-orientation_estimator.attitude_b_g.RData")

