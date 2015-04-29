###############################################
# Post Processing R Script to compute         
# the Allan variance over the FOG KVH DSP3000            
# Javier Hidalgo Carrió
# DFKI-RIC May 2011                            
###############################################

#libraries
library(allanvar)
D2R<-pi/180.00
R2D<-180.00/pi

####################################################################################################################################
#Change the path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110527_CG_JH_xsens_and_FOG+temp_all_night/")

#############
# FOG
#############

#### FOG ####
values <- read.table ("fog_zaxis.csv", sep=" ")

names(values) = c("time", "gyroz")

str(values)
delta <- NULL;

#Time is in microseconds (10^-6)
delta[1] = (values$time[10]-values$time[9])/1000000
delta[2] = (values$time[100]-values$time[99])/1000000
delta[3] = (values$time[1000]-values$time[999])/1000000
delta[4] = (values$time[6]-values$time[5])/1000000

# Movementes at the end of the data record (because someone moved Dagon)
# Therefore, it taked out the last 2 hours (662400 samples)
########################## X Axis ##########################
fogGyro <- ts (data.frame(values$gyroz[1:4270113]), delta=mean(delta))

plot(fogGyro)
lines (rep(mean(fogGyro), length(fogGyro)), xlab="", ylab="", col="red")
lines (rep(mean(fogGyro)-sd(fogGyro), length(fogGyro)), xlab="", ylab="", col="blue")
lines (rep(mean(fogGyro)+sd(fogGyro), length(fogGyro)), xlab="", ylab="", col="blue")

meanfog <- mean (data.frame(values$gyroz[1:4270113]))

rm (values)

####################################################################################################################################
#Change the path
setwd ("/home/likewise-open/DFKI/jhidalgocarrio/CUSLAM/experiments/20110527_CG_JH_xsens_and_FOG+temp_all_night/")
#load("xsens_sensors.Rdata")

#############
# XSENS ACC
#############
values <- read.table ("xsens_acc.csv", sep=" ")

names(values) = c("time", "accx", "accy", "accz")

str(values)

#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000

v <- NULL

########################## X Axis ##########################
v[1] = mean (data.frame(values$accx[1:4270113]))

########################## Y Axis ##########################
v[2] = mean (data.frame(values$accy[1:4270113]))

########################## Z Axis ##########################
v[3] = mean (data.frame(values$accz[1:4270113]))

rm (values)

#############
# XSENS GYRO
#############

values <- read.table ("xsens_gyro.csv", sep=" ")

names(values) = c("time", "gyrox", "gyroy", "gyroz")

str(values)
#Time is in microseconds (10^-6)
delta = (values$time[2]-values$time[1])/1000000

u <-NULL

########################## X Axis ##########################
u[1] = mean(data.frame(values$gyrox[1:4270113]))

########################## Y Axis ##########################
u[2] = mean(data.frame(values$gyroy[1:4270113]))

########################## Z Axis ##########################
u[3] = mean(data.frame(values$gyroz[1:4270113]))

rm (values)

##############################
### Compute Pitch and Roll ###
##############################

g <- sqrt(v[1]^2 + v[2]^2 + v[3]^2)

euler <-NULL

euler[1] = 140.0*D2R #YAW 
euler[2] =  asin((v[2])/g); #ROLL 
euler[3] =  -atan((v[1])/(v[3])) #PITCH


##############################
### Euler to quaternion    ###
##############################
c1 = cos(euler[1]/2);
s1 = sin(euler[1]/2);
c2 = cos(euler[2]/2);
s2 = sin(euler[2]/2);
c3 = cos(euler[3]/2);
s3 = sin(euler[3]/2);

q <- NULL
q[4] = (c1*c2*c3 + s1*s2*s3)
q[1] = (c1*c2*s3 - s1*s2*c3)
q[2] = (c1*s2*c3 + s1*c2*s3)
q[3] = (s1*c2*c3 - c1*s2*s3)



##############################
### Quaternion to DCM      ###
##############################

DCM11 = q[4]^2+q[1]^2-q[2]^2-q[3]^2

DCM12 = 2*((q[1]*q[2]) - (q[4]*q[3]))

DCM13 = 2*((q[1]*q[3]) + (q[4]*q[2]))

DCM21 = 2*((q[1]*q[2]) + (q[4]*q[3]))

DCM22 = q[4]^2-q[1]^2+q[2]^2-q[3]^2

DCM23 = 2*((q[2]*q[3]) - (q[4]*q[1]))

DCM31 = 2*((q[1]*q[3]) - (q[4]*q[2]))

DCM32 = 2*((q[2]*q[3]) + (q[4]*q[1]))

DCM33 = q[4]^2-q[1]^2-q[2]^2+q[3]^2

DCM <- matrix (c(DCM11, DCM12, DCM13, DCM21,  DCM22, DCM23,  DCM31, DCM32, DCM33), nrow=3, ncol=3)

##############################
### Earth angular velocity ###
##############################
w <-NULL
w[1] =  7.292115e-05 * cos (0.926478944)
w[2] = 0
w[3] =  7.292115e-05 * sin (0.926478944)


##############################
### Vector transformation  ###
##############################
w2 <-NULL
w2 <- DCM%*%w


##############################
## Substract Earth Rotation ##
##############################
ang = c(v[1], v[2], meanfog)

## Gyroscopes bias offset ##
w3 <- ang - w2

