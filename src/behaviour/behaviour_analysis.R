###############################################
# Post Processing R Script to compute         
# the Phase Space of a robot motion
# Javier Hidalgo Carrió
# DFKI-RIC October 2011                            
###############################################


################
# ASQUARG Test 1
################
# You may change this path
setwd ("~/iMoby/experiments/asguard_loop/20111024-1353")
load ("pose_estimator.0.log-pose_estimator.pose_samples.RData")

ls()
time <- as.data.frame(time)
names(time) <- seq(1, length(time))

position <- as.data.frame(position)
names(position) <- seq(1, length(position))

orientation <- as.data.frame(orientation)
names(orientation) <- seq(1, length(orientation))


#Data time adaptation
sec <- as.integer(sapply(time,function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t1 <- as.numeric(paste(sec, usec, sep=''))

delta1 = NULL
for (i in 1:(length(t1)-1))
{
  delta1[i] <- (t1[i+1] - t1[i])/1000000
}

pos1 <- NULL
pos1$x <- unlist(position [1, ])
pos1$y <- unlist(position [2, ])
pos1$z <- unlist(position [3, ])
pos1$yaw <- unlist(orientation [3, ])

#Ploting the values
library(scatterplot3d)
scatterplot3d (pos1$x, pos1$y, pos1$z, type="l")

plot (pos1$x, type="l")
plot (pos1$y, type="l")
plot (pos1$z, type="l")
plot (pos1$yaw, type="l")

library(rgl)
open3d()
text3d(x,y,z,labels)
plot3d(pos1$x,pos1$y, pos1$z, type='l')


################
# ASQUARG Test 2
################
# You may change this path
setwd ("~/iMoby/experiments/asguard_loop/20111024-1415")
load ("pose_estimator.0.log-pose_estimator.pose_samples.RData")

ls()
time <- as.data.frame(time)
names(time) <- seq(1, length(time))

position <- as.data.frame(position)
names(position) <- seq(1, length(position))

orientation <- as.data.frame(orientation)
names(orientation) <- seq(1, length(orientation))


#Data time adaptation
sec <- as.integer(sapply(time,function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t2 <- as.numeric(paste(sec, usec, sep=''))

delta2 = NULL
for (i in 1:(length(t2)-1))
{
  delta2[i] <- (t2[i+1] - t2[i])/1000000
}

pos2 <- NULL
pos2$x <- unlist(position [1, ])
pos2$y <- unlist(position [2, ])
pos2$z <- unlist(position [3, ])
pos2$yaw <- unlist(orientation [3, ])

#Ploting the values
library(scatterplot3d)
scatterplot3d (pos2$x, pos2$y, pos2$z, type="l")


################
# ASQUARG Test 3
################
# You may change this path
setwd ("~/iMoby/experiments/asguard_loop/20111024-1431")
load ("pose_estimator.0.log-pose_estimator.pose_samples.RData")

ls()
time <- as.data.frame(time)
names(time) <- seq(1, length(time))

position <- as.data.frame(position)
names(position) <- seq(1, length(position))

orientation <- as.data.frame(orientation)
names(orientation) <- seq(1, length(orientation))


#Data time adaptation
sec <- as.integer(sapply(time,function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t3 <- as.numeric(paste(sec, usec, sep=''))

delta3 = NULL
for (i in 1:(length(t3)-1))
{
  delta3[i] <- (t3[i+1] - t3[i])/1000000
}

pos3 <- NULL
pos3$x <- unlist(position [1, ])
pos3$y <- unlist(position [2, ])
pos3$z <- unlist(position [3, ])
pos3$yaw <- unlist(orientation [3, ])

#Ploting the values
library(scatterplot3d)
scatterplot3d (pos3$x, pos3$y, pos3$z, type="l")

plot (pos$x, type="l")
plot (pos3$y, type="l")
plot (pos3$z, type="l")
plot (pos3$yaw, type="l")

library(rgl)
open3d()
text3d(x,y,z,labels)
plot3d(pos3$x,pos3$y, pos3$z, type='l')

################
# ASQUARG Test 4
################
# You may change this path
setwd ("~/iMoby/experiments/asguard_loop/20111024-1451")
load ("pose_estimator.0.log-pose_estimator.pose_samples.RData")

ls()
time <- as.data.frame(time)
names(time) <- seq(1, length(time))

position <- as.data.frame(position)
names(position) <- seq(1, length(position))

orientation <- as.data.frame(orientation)
names(orientation) <- seq(1, length(orientation))


#Data time adaptation
sec <- as.integer(sapply(time,function(x) x[1]))
usec <- as.integer(sapply(time,function(x) x[2]))

t4 <- as.numeric(paste(sec, usec, sep=''))

delta4 = NULL
for (i in 1:(length(t4)-1))
{
  delta4[i] <- (t4[i+1] - t4[i])/1000000
}

pos4 <- NULL
pos4$x <- unlist(position [1, ])
pos4$y <- unlist(position [2, ])
pos4$z <- unlist(position [3, ])
pos4$yaw <- unlist(orientation [3, ])

#Ploting the values
library(scatterplot3d)
scatterplot3d (pos4$x, pos4$y, pos4$z, type="l")

plot (pos$x, type="l")
plot (pos4$y, type="l")
plot (pos4$z, type="l")
plot (pos4$yaw, type="l")

library(rgl)
open3d()
text3d(x,y,z,labels)
plot3d(pos4$x,pos4$y, pos4$z, type='l')


########################
# MERGER ALL THE TESTS
# IN ONE DATA
########################

t <- as.numeric(c (t1, t2, t3, t4))
pos <- NULL
pos$x <- as.numeric(c (pos1$x, pos2$x, pos3$x, pos4$x))
pos$y <- as.numeric(c (pos1$y, pos2$y, pos3$y, pos4$y))
pos$z <- as.numeric(c (pos1$z, pos2$z, pos3$z, pos4$z))
pos$yaw <- as.numeric(c (pos1$yaw, pos2$yaw, pos3$yaw, pos4$yaw))  


########################
# PLOTING ALL THE TESTS
# IN ONE GRAPH
########################
plot (pos1$x, type="l", col="red")
lines (pos2$x, type="l", col="blue")
lines (pos3$x, type="l", col="green")
lines (pos4$x, type="l", col="orange")

plot (pos1$y, type="l", col="red")
lines (pos2$y, type="l", col="blue")
lines (pos3$y, type="l", col="green")
lines (pos4$y, type="l", col="orange")

plot (pos1$z, type="l", col="red")
lines (pos2$z, type="l", col="blue")
lines (pos3$z, type="l", col="green")
lines (pos4$z, type="l", col="orange")

plot (pos1$yaw, type="l", col="red")
lines (pos2$yaw, type="l", col="blue")
lines (pos3$yaw, type="l", col="green")
lines (pos4$yaw, type="l", col="orange")

png(filename = "asguard_trajectory.png", width=1024, height=768, units = "px", pointsize = 12, bg = "white", res = NA)
plot (pos1$x, pos1$y, type="l", col="red", asp= 1, xlab="", ylab="")
title(main = "Asguard - Trajectory follower path", xlab = "x coordinate (meters)", ylab = "y coordinate (meters)")
lines (pos2$x, pos2$y,col="blue")
lines (pos3$x, pos3$y,col="green")
lines (pos4$x, pos4$y,col="orange")
grid(col="black")
dev.off()

library(rgl)
open3d()
text3d(x,y,z,labels)
plot3d(pos1$x,pos1$y, pos1$z, type='l')
lines3d(pos2$x,pos2$y, pos2$z, col='blue')
lines3d(pos3$x,pos3$y, pos3$z, col='green')
lines3d(pos4$x,pos4$y, pos4$z, col='orange')


########################
# CONSTRUCT THE ATRACTOR
########################
#libraries
library(RTisean)
library(tseriesChaos)


delta = (mean(delta1))


##Embedding dimension
## Takens' theorem p > 2n where n is the dimension of the attractor (degrees of freedom)
p=5


##Embedding lag (timedelay)
tsx <- ts (data.frame(pos$x), delta=mean(delta))
autotsx <- acf(tsx,  lag.max=5000)
tau=3400 ##autotsx$acf[3400] = exp(-1) = 0.37

##space-time separation plot, which can
##be used to decide the Theiler time window t.
stplot (tsx, m=p, d=tau, idt= 23119, mdt=11) ##length(pos4$y)

##Nearest Neighbours to help deciding the embedding dimension
## t -> Theiler window, minimal temporal separation of neighbours
##when this number (nearest neighbours) is near zero, the attractor is properly unfolded and contains no
##self-intersections
tsx.out <- false.nearest(tsx, m=40, d=773, t=1200, eps=4, rt=3)
plot (tsx.out)

##With Mutual information
muttsx <- mutual(tsx, lag.max = 850) ## at muttsx[773] is the first minimum
tau=773


##Compute the embedding-lag reconstructor
at <- embedd(tsx, m=p, d=tau)
scatterplot3d(at, type="l", color="red")

library(rgl)
open3d()
text3d(x,y,z,labels)
plot3d(at, type='l', co="red")


##Compute the Lyapunov exponent
lyapat <-lyap_k(tsx, m=p, d=tau, s=800, t=40, ref=1700, k=4, eps=4)
plot(lyapat)
lyap(lyapat, 0.73, 2.47)


save.image(file = "behaviour_asguardloop.RData", compress = TRUE)

