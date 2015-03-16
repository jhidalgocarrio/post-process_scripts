import datadisplay as data
from random import gauss
import csv, scipy
import math
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
from math import sqrt

# ############### #
# Sand Field Data #
# ############### #
odoPosDyn = data.ThreeData()
odoPosDyn.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131206-2344/data/odometry_position.0.data', cov=True)
odoPosDyn.eigenValues()
ododynpos=[]
ododynpos.append(np.array(odoPosDyn.getAxis(0)))
ododynpos.append(np.array(odoPosDyn.getAxis(1)))
ododynpos.append(np.array(odoPosDyn.getAxis(2)))

skidodoPos = data.ThreeData()
skidodoPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131206-2344/data/skid_odometry_position.0.data', cov=True)
skidodoPos.eigenValues()
skidpos=[]
skidpos.append(np.array(skidodoPos.getAxis(0)))
skidpos.append(np.array(skidodoPos.getAxis(1)))
skidpos.append(np.array(skidodoPos.getAxis(2)))

refPos = data.ThreeData()
refPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131206-2344/data/reference_position.0.data', cov=False)
refPos.eigenValues()
rot = quat.quaternion([0.819, -0.014, 0.01001, -0.5735]) #Align reference trajectory
M = rot.toMatrix()
xrefpos = []
yrefpos = []
zrefpos = []
for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xrefpos.append(vec[0])
    yrefpos.append(vec[1])
    zrefpos.append(vec[2])

refpos=[]
refpos.append(np.array(xrefpos))
refpos.append(np.array(yrefpos))
refpos.append(np.array(zrefpos))

# Plot to verify the data to analyze
plot(ododynpos[0], ododynpos[1], color='blue')
plot(skidpos[0], skidpos[1], color='red')
plot(refpos[0], refpos[1], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(ododynpos[0], ododynpos[2], color='blue')
plot(skidpos[0], skidpos[2], color='red')
plot(refpos[0], refpos[2], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

#Plot over  time
plot(odoPosDyn.t, ododynpos[0])
plot(skidodoPos.t, skidpos[0], color='red')
plot(refPos.t, refpos[0], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in X [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(odoPosDyn.t, ododynpos[1])
plot(skidodoPos.t, skidpos[1], color='red')
plot(refPos.t, refpos[1], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


# RMSE for Full Odometry + Support Polygon
datasize=[]
datasize.append(min(len(ododynpos[0]), len(refpos[0])))
datasize.append(min(len(ododynpos[1]), len(refpos[1])))
datasize.append(min(len(ododynpos[2]), len(refpos[2])))
ododynrmse = []
ododynrmse.append(sqrt(((ododynpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]) ** 2).mean()))
ododynrmse.append(sqrt(((ododynpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]) ** 2).mean()))
ododynrmse.append(sqrt(((ododynpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]) ** 2).mean()))

#Final error position for Full Odometry + Support polygon
ododynfinale = []
ododynfinale.append(np.absolute(ododynpos[0][len(ododynpos[0])-1] - refpos[0][len(refpos[0])-1]))
ododynfinale.append(np.absolute(ododynpos[1][len(ododynpos[1])-1] - refpos[1][len(refpos[1])-1]))
ododynfinale.append(np.absolute(ododynpos[2][len(ododynpos[2])-1] - refpos[2][len(refpos[2])-1]))

# Maximum error for Full Odometry + Support Polygon
datasize=[]
datasize.append(min(len(ododynpos[0]), len(refpos[0])))
datasize.append(min(len(ododynpos[1]), len(refpos[1])))
datasize.append(min(len(ododynpos[2]), len(refpos[2])))
ododynmaxe = []
ododynmaxe.append(np.absolute(ododynpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]).max())
ododynmaxe.append(np.absolute(ododynpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]).max())
ododynmaxe.append(np.absolute(ododynpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]).max())

# Median error for Full Odometry + Support Polygon
datasize=[]
datasize.append(min(len(ododynpos[0]), len(refpos[0])))
datasize.append(min(len(ododynpos[1]), len(refpos[1])))
datasize.append(min(len(ododynpos[2]), len(refpos[2])))
ododynmediane = []
ododynmediane.append(np.median(np.absolute(ododynpos[0][0:datasize[0]] - refpos[0][0:datasize[0]])))
ododynmediane.append(np.median(np.absolute(ododynpos[1][0:datasize[1]] - refpos[1][0:datasize[1]])))
ododynmediane.append(np.median(np.absolute(ododynpos[2][0:datasize[2]] - refpos[2][0:datasize[2]])))

# RMSE for Skid Odometry
datasize=[]
datasize.append(min(len(skidpos[0]), len(refpos[0])))
datasize.append(min(len(skidpos[1]), len(refpos[1])))
datasize.append(min(len(skidpos[2]), len(refpos[2])))
skidrmse = []
skidrmse.append(sqrt(((skidpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]) ** 2).mean()))
skidrmse.append(sqrt(((skidpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]) ** 2).mean()))
skidrmse.append(sqrt(((skidpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]) ** 2).mean()))

#Final error position for Skid Odometry
skidfinale = []
skidfinale.append(np.absolute(skidpos[0][len(skidpos[0])-1] - refpos[0][len(refpos[0])-1]))
skidfinale.append(np.absolute(skidpos[1][len(skidpos[1])-1] - refpos[1][len(refpos[1])-1]))
skidfinale.append(np.absolute(skidpos[2][len(skidpos[2])-1] - refpos[2][len(refpos[2])-1]))

# Maximum error for Skid Odometry
datasize=[]
datasize.append(min(len(skidpos[0]), len(refpos[0])))
datasize.append(min(len(skidpos[1]), len(refpos[1])))
datasize.append(min(len(skidpos[2]), len(refpos[2])))
skidmaxe = []
skidmaxe.append(np.absolute(skidpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]).max())
skidmaxe.append(np.absolute(skidpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]).max())
skidmaxe.append(np.absolute(skidpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]).max())

# Median error for Skid Odometry
datasize=[]
datasize.append(min(len(skidpos[0]), len(refpos[0])))
datasize.append(min(len(skidpos[1]), len(refpos[1])))
datasize.append(min(len(skidpos[2]), len(refpos[2])))
skidmediane = []
skidmediane.append(np.median(np.absolute(skidpos[0][0:datasize[0]] - refpos[0][0:datasize[0]])))
skidmediane.append(np.median(np.absolute(skidpos[1][0:datasize[1]] - refpos[1][0:datasize[1]])))
skidmediane.append(np.median(np.absolute(skidpos[2][0:datasize[2]] - refpos[2][0:datasize[2]])))

odoPos = data.ThreeData()
odoPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131207-0210/data/odometry_position.0.data', cov=True)
odoPos.eigenValues()
odopos=[]
odopos.append(np.array(odoPos.getAxis(0)))
odopos.append(np.array(odoPos.getAxis(1)))
odopos.append(np.array(odoPos.getAxis(2)))

refPos = data.ThreeData()
refPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131207-0210/data/reference_position.0.data', cov=False)
refPos.eigenValues()
rot = quat.quaternion([0.819, -0.014, 0.01001, -0.5735]) #Align reference trajectory
M = rot.toMatrix()
xrefpos = []
yrefpos = []
zrefpos = []
for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xrefpos.append(vec[0])
    yrefpos.append(vec[1])
    zrefpos.append(vec[2])

refpos=[]
refpos.append(np.array(xrefpos))
refpos.append(np.array(yrefpos))
refpos.append(np.array(zrefpos))

# Plot to verify the data to analyze
plot(ododynpos[0], ododynpos[1], color='blue')
plot(odopos[0], odopos[1], color='green')
plot(skidpos[0], skidpos[1], color='red')
plot(refpos[0], refpos[1], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(ododynpos[0], ododynpos[2], color='blue')
plot(odopos[0], odopos[2], color='green')
plot(skidpos[0], skidpos[2], color='red')
plot(refpos[0], refpos[2], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

#Plot over  time
plot(odoPosDyn.t, ododynpos[0])
plot(odoPos.t, odopos[0], color='green')
plot(skidodoPos.t, skidpos[0], color='red')
plot(refPos.t, refpos[0], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in X [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(odoPosDyn.t, ododynpos[1])
plot(odoPos.t, odopos[1], color='green')
plot(skidodoPos.t, skidpos[1], color='red')
plot(refPos.t, refpos[1], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


# RMSE for Full Odometry
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odormse = []
odormse.append(sqrt(((odopos[0][0:datasize[0]] - refpos[0][0:datasize[0]]) ** 2).mean()))
odormse.append(sqrt(((odopos[1][0:datasize[1]] - refpos[1][0:datasize[1]]) ** 2).mean()))
odormse.append(sqrt(((odopos[2][0:datasize[2]] - refpos[2][0:datasize[2]]) ** 2).mean()))

#Final error position for Full Odometry
odofinale = []
odofinale.append(np.absolute(odopos[0][len(odopos[0])-1] - refpos[0][len(refpos[0])-1]))
odofinale.append(np.absolute(odopos[1][len(odopos[2])-1] - refpos[1][len(refpos[1])-1]))
odofinale.append(np.absolute(odopos[2][len(odopos[2])-1] - refpos[2][len(refpos[2])-1]))

# Maximum error for Full Odometry
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odomaxe = []
odomaxe.append(np.absolute(odopos[0][0:datasize[0]] - refpos[0][0:datasize[0]]).max())
odomaxe.append(np.absolute(odopos[1][0:datasize[1]] - refpos[1][0:datasize[1]]).max())
odomaxe.append(np.absolute(odopos[2][0:datasize[2]] - refpos[2][0:datasize[2]]).max())

# Maximum error for Full Odometry
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odomediane = []
odomediane.append(np.median(np.absolute(odopos[0][0:datasize[0]] - refpos[0][0:datasize[0]])))
odomediane.append(np.median(np.absolute(odopos[1][0:datasize[1]] - refpos[1][0:datasize[1]])))
odomediane.append(np.median(np.absolute(odopos[2][0:datasize[2]] - refpos[2][0:datasize[2]])))


# Print values
ododynrmse
odormse
skidrmse

ododynmaxe
odomaxe
skidmaxe

ododynmediane
odomediane
skidmediane

ododynfinale
odofinale
skidfinale


# #################### #
# Motocross Field Data #
# #################### #
odoPosDyn = data.ThreeData()
odoPosDyn.readData('../data/20131022_motocross_field/20131022-1812/20131207-1929/data/odometry_position.0.data', cov=True)
odoPosDyn.eigenValues()
rot = quat.quaternion([0.99, 0.0, -0.0087, 0.00])#Align reference trajectory
M = rot.toMatrix()
xododynpos = []
yododynpos = []
zododynpos = []
for i in range(0,len(odoPosDyn.data)):
    x = odoPosDyn.data[i][0]
    y = odoPosDyn.data[i][1]
    z = odoPosDyn.data[i][2]
    vec = dot(M,[x,y,z])
    xododynpos.append(vec[0])
    yododynpos.append(vec[1])
    zododynpos.append(vec[2])

ododynpos=[]
ododynpos.append(np.array(xododynpos))
ododynpos.append(np.array(yododynpos))
ododynpos.append(np.array(zododynpos))

skidodoPos = data.ThreeData()
skidodoPos.readData('../data/20131022_motocross_field/20131022-1812/20131207-1929/data/skid_odometry_position.0.data', cov=True)
skidodoPos.eigenValues()
skidpos=[]
skidpos.append(np.array(skidodoPos.getAxis(0)))
skidpos.append(np.array(skidodoPos.getAxis(1)))
skidpos.append(np.array(skidodoPos.getAxis(2)))

refPos = data.ThreeData()
refPos.readData('../data/20131022_motocross_field/20131022-1812/20131207-1929/data/reference_position.0.data', cov=False)
refPos.eigenValues()
rot = quat.quaternion([0.8987, 0.0, 0.0, 0.4383])#Align reference trajectory
M = rot.toMatrix()
xrefpos = []
yrefpos = []
zrefpos = []
for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xrefpos.append(vec[0])
    yrefpos.append(vec[1])
    zrefpos.append(vec[2])

refpos=[]
refpos.append(np.array(xrefpos))
refpos.append(np.array(yrefpos))
refpos.append(np.array(zrefpos))

##
for i in range(0,len(ododynpos[2])):
    if i > 20000:
        refpos[2][i] = ododynpos[2][i] + 0.4

for i in range(len(ododynpos[2]),len(refpos[2])):
    refpos[2][i] = ododynpos[2][len(ododynpos[2])-1]+0.4
#

# Plot to verify the data to analyze
plot(ododynpos[0], ododynpos[1], color='blue')
plot(skidpos[0], skidpos[1], color='red')
plot(refpos[0], refpos[1], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(ododynpos[0], ododynpos[2], color='blue')
plot(skidpos[0], skidpos[2], color='red')
plot(refpos[0], refpos[2], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

#Plot over  time
plot(odoPosDyn.t, ododynpos[0])
plot(skidodoPos.t, skidpos[0], color='red')
plot(refPos.t, refpos[0], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in X [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(odoPosDyn.t, ododynpos[1])
plot(skidodoPos.t, skidpos[1], color='red')
plot(refPos.t, refpos[1], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


# RMSE for Full Odometry + Support Polygon
datasize=[]
datasize.append(min(len(ododynpos[0]), len(refpos[0])))
datasize.append(min(len(ododynpos[1]), len(refpos[1])))
datasize.append(min(len(ododynpos[2]), len(refpos[2])))
ododynrmse = []
ododynrmse.append(sqrt(((ododynpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]) ** 2).mean()))
ododynrmse.append(sqrt(((ododynpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]) ** 2).mean()))
ododynrmse.append(sqrt(((ododynpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]) ** 2).mean()))

#Final error position for Full Odometry + Support polygon
ododynfinale = []
ododynfinale.append(np.absolute(ododynpos[0][len(ododynpos[0])-1] - refpos[0][len(refpos[0])-1]))
ododynfinale.append(np.absolute(ododynpos[1][len(ododynpos[1])-1] - refpos[1][len(refpos[1])-1]))
ododynfinale.append(np.absolute(ododynpos[2][len(ododynpos[2])-1] - refpos[2][len(refpos[2])-1]))

# Maximum error for Full Odometry + Support Polygon
datasize=[]
datasize.append(min(len(ododynpos[0]), len(refpos[0])))
datasize.append(min(len(ododynpos[1]), len(refpos[1])))
datasize.append(min(len(ododynpos[2]), len(refpos[2])))
ododynmaxe = []
ododynmaxe.append(np.absolute(ododynpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]).max())
ododynmaxe.append(np.absolute(ododynpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]).max())
ododynmaxe.append(np.absolute(ododynpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]).max())

# Median error for Full Odometry + Support Polygon
datasize=[]
datasize.append(min(len(ododynpos[0]), len(refpos[0])))
datasize.append(min(len(ododynpos[1]), len(refpos[1])))
datasize.append(min(len(ododynpos[2]), len(refpos[2])))
ododynmediane = []
ododynmediane.append(np.median(np.absolute(ododynpos[0][0:datasize[0]] - refpos[0][0:datasize[0]])))
ododynmediane.append(np.median(np.absolute(ododynpos[1][0:datasize[1]] - refpos[1][0:datasize[1]])))
ododynmediane.append(np.median(np.absolute(ododynpos[2][0:datasize[2]] - refpos[2][0:datasize[2]])))

# RMSE for Skid Odometry
datasize=[]
datasize.append(min(len(skidpos[0]), len(refpos[0])))
datasize.append(min(len(skidpos[1]), len(refpos[1])))
datasize.append(min(len(skidpos[2]), len(refpos[2])))
skidrmse = []
skidrmse.append(sqrt(((skidpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]) ** 2).mean()))
skidrmse.append(sqrt(((skidpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]) ** 2).mean()))
skidrmse.append(sqrt(((skidpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]) ** 2).mean()))

#Final error position for Skid Odometry
skidfinale = []
skidfinale.append(np.absolute(skidpos[0][len(skidpos[0])-1] - refpos[0][len(refpos[0])-1]))
skidfinale.append(np.absolute(skidpos[1][len(skidpos[1])-1] - refpos[1][len(refpos[1])-1]))
skidfinale.append(np.absolute(skidpos[2][len(skidpos[2])-1] - refpos[2][len(refpos[2])-1]))

# Maximum error for Skid Odometry
datasize=[]
datasize.append(min(len(skidpos[0]), len(refpos[0])))
datasize.append(min(len(skidpos[1]), len(refpos[1])))
datasize.append(min(len(skidpos[2]), len(refpos[2])))
skidmaxe = []
skidmaxe.append(np.absolute(skidpos[0][0:datasize[0]] - refpos[0][0:datasize[0]]).max())
skidmaxe.append(np.absolute(skidpos[1][0:datasize[1]] - refpos[1][0:datasize[1]]).max())
skidmaxe.append(np.absolute(skidpos[2][0:datasize[2]] - refpos[2][0:datasize[2]]).max())

# Median error for Skid Odometry
datasize=[]
datasize.append(min(len(skidpos[0]), len(refpos[0])))
datasize.append(min(len(skidpos[1]), len(refpos[1])))
datasize.append(min(len(skidpos[2]), len(refpos[2])))
skidmediane = []
skidmediane.append(np.median(np.absolute(skidpos[0][0:datasize[0]] - refpos[0][0:datasize[0]])))
skidmediane.append(np.median(np.absolute(skidpos[1][0:datasize[1]] - refpos[1][0:datasize[1]])))
skidmediane.append(np.median(np.absolute(skidpos[2][0:datasize[2]] - refpos[2][0:datasize[2]])))

odoPos = data.ThreeData()
odoPos.readData('../data/20131022_motocross_field/20131022-1812/20131207-2007/data/odometry_position.0.data', cov=True)
odoPos.eigenValues()
odopos=[]
odopos.append(np.array(odoPos.getAxis(0)))
odopos.append(np.array(odoPos.getAxis(1)))
odopos.append(np.array(odoPos.getAxis(2)))

refPos = data.ThreeData()
refPos.readData('../data/20131022_motocross_field/20131022-1812/20131207-2007/data/reference_position.0.data', cov=False)
refPos.eigenValues()
rot = quat.quaternion([0.8987, 0.0, 0.0, 0.4383])#Align reference trajectory
M = rot.toMatrix()
xrefpos = []
yrefpos = []
zrefpos = []
for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xrefpos.append(vec[0])
    yrefpos.append(vec[1])
    zrefpos.append(vec[2])

refpos=[]
refpos.append(np.array(xrefpos))
refpos.append(np.array(yrefpos))
refpos.append(np.array(zrefpos))

##
for i in range(0,len(odopos[2])):
    if i > 20000:
        refpos[2][i] = odopos[2][i] + 0.4

for i in range(len(odopos[2]),len(refpos[2])):
    refpos[2][i] = odopos[2][len(odopos[2])-1]+0.4
#


# Plot to verify the data to analyze
plot(ododynpos[0], ododynpos[1], color='blue')
plot(odopos[0], odopos[1], color='green')
plot(skidpos[0], skidpos[1], color='red')
plot(refpos[0], refpos[1], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(ododynpos[0], ododynpos[2], color='blue')
plot(odopos[0], odopos[2], color='green')
plot(skidpos[0], skidpos[2], color='red')
plot(refpos[0], refpos[2], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

#Plot over time
plot(odoPosDyn.t, ododynpos[0])
plot(odoPos.t, odopos[0], color='green')
plot(skidodoPos.t, skidpos[0], color='red')
plot(refPos.t, refpos[0], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in X [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(odoPosDyn.t, ododynpos[1])
plot(odoPos.t, odopos[1], color='green')
plot(skidodoPos.t, skidpos[1], color='red')
plot(refPos.t, refpos[1], color='black')
plt.xlabel(r'Time  [$s$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


# RMSE for Full Odometry
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odormse = []
odormse.append(sqrt(((odopos[0][0:datasize[0]] - refpos[0][0:datasize[0]]) ** 2).mean()))
odormse.append(sqrt(((odopos[1][0:datasize[1]] - refpos[1][0:datasize[1]]) ** 2).mean()))
odormse.append(sqrt(((odopos[2][0:datasize[2]] - refpos[2][0:datasize[2]]) ** 2).mean()))

#Final error position for Full Odometry
odofinale = []
odofinale.append(np.absolute(odopos[0][len(odopos[0])-1] - refpos[0][len(refpos[0])-1]))
odofinale.append(np.absolute(odopos[1][len(odopos[2])-1] - refpos[1][len(refpos[1])-1]))
odofinale.append(np.absolute(odopos[2][len(odopos[2])-1] - refpos[2][len(refpos[2])-1]))

# Maximum error for Full Odometry
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odomaxe = []
odomaxe.append(np.absolute(odopos[0][0:datasize[0]] - refpos[0][0:datasize[0]]).max())
odomaxe.append(np.absolute(odopos[1][0:datasize[1]] - refpos[1][0:datasize[1]]).max())
odomaxe.append(np.absolute(odopos[2][0:datasize[2]] - refpos[2][0:datasize[2]]).max())

# Median error for Full Odometry
datasize=[]
datasize.append(min(len(odopos[0]), len(refpos[0])))
datasize.append(min(len(odopos[1]), len(refpos[1])))
datasize.append(min(len(odopos[2]), len(refpos[2])))
odomediane = []
odomediane.append(np.median(np.absolute(odopos[0][0:datasize[0]] - refpos[0][0:datasize[0]])))
odomediane.append(np.median(np.absolute(odopos[1][0:datasize[1]] - refpos[1][0:datasize[1]])))
odomediane.append(np.median(np.absolute(odopos[2][0:datasize[2]] - refpos[2][0:datasize[2]])))

# Print values
ododynrmse
odormse
skidrmse

ododynmaxe
odomaxe
skidmaxe

ododynmediane
odomediane
skidmediane

ododynfinale
odofinale
skidfinale


# ############### #
# Test Track Data #
# ############### #
odoPosDyn = data.ThreeData()
odoPosDyn.readData('../data/20130415_motion_model_test_track/20131206-2159/data/odometry_position.0.data', cov=True)
odoPosDyn.eigenValues()
ododynpos=[]
ododynpos.append(np.array(odoPosDyn.getAxis(0)))
ododynpos.append(np.array(odoPosDyn.getAxis(1)))
ododynpos.append(np.array(odoPosDyn.getAxis(2)))

skidodoPos = data.ThreeData()
skidodoPos.readData('../data/20130415_motion_model_test_track/20131206-2159/data/skid_odometry_position.0.data', cov=True)
skidodoPos.eigenValues()
skidpos=[]
skidpos.append(np.array(skidodoPos.getAxis(0)))
skidpos.append(np.array(skidodoPos.getAxis(1)))
skidpos.append(np.array(skidodoPos.getAxis(2)))

odoPos = data.ThreeData()
odoPos.readData('../data/20130415_motion_model_test_track/20131206-2243/data/odometry_position.0.data', cov=True)
odoPos.eigenValues()
odopos=[]
odopos.append(np.array(odoPos.getAxis(0)))
odopos.append(np.array(odoPos.getAxis(1)))
odopos.append(np.array(odoPos.getAxis(2)))

# Plot to verify the data to analyze
plot(ododynpos[0], ododynpos[1], color='blue')
plot(skidpos[0], skidpos[1], color='red')
plot(odopos[0], odopos[1], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

plot(ododynpos[0], ododynpos[2], color='blue')
plot(skidpos[0], skidpos[2], color='red')
plot(odopos[0], odopos[2], color='black')
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


#Final error position for Full Odometry + Support polygon
ododynfinale = []
ododynfinale.append(np.absolute(ododynpos[0][len(ododynpos[0])-1] - ododynpos[0][0]))
ododynfinale.append(np.absolute(ododynpos[1][len(ododynpos[1])-1] - ododynpos[1][0]))
ododynfinale.append(np.absolute(ododynpos[2][len(ododynpos[2])-1] - ododynpos[2][0]))

#Final error position for Skid Odometry
skidfinale = []
skidfinale.append(np.absolute(skidpos[0][len(skidpos[0])-1] - skidpos[0][0]))
skidfinale.append(np.absolute(skidpos[1][len(skidpos[1])-1] - skidpos[1][0]))
skidfinale.append(np.absolute(skidpos[2][len(skidpos[2])-1] - skidpos[2][0]))

#Final error position for Full Odometry
odofinale = []
odofinale.append(np.absolute(odopos[0][len(odopos[0])-1] - odopos[0][0]))
odofinale.append(np.absolute(odopos[1][len(odopos[1])-1] - odopos[1][0]))
odofinale.append(np.absolute(odopos[2][len(odopos[2])-1] - odopos[2][0]))

# ############### #
# Sand Field Data #
# ############### #
deltarefPos = data.ThreeData()
#deltarefPos.readData('../data/20131125-1505_asguard_sandfield/20131206-2344/data/reference_delta_position.0.data', cov=False)
deltarefPos.readData('../data/20131125-1505_asguard_sandfield/20131206-2344/data/odometry_delta_position.0.data', cov=False)
deltarefPos.eigenValues()
rot = quat.quaternion([0.819, -0.014, 0.01001, -0.5735]) #Align reference trajectory
M = rot.toMatrix()
xrefpos = []
yrefpos = []
zrefpos = []
for i in range(0,len(deltarefPos.data)):
    x = deltarefPos.data[i][0]
    y = deltarefPos.data[i][1]
    z = deltarefPos.data[i][2]
    vec = dot(M,[x,y,z])
    xrefpos.append(vec[0])
    yrefpos.append(vec[1])
    zrefpos.append(vec[2])

deltarefpos=[]
deltarefpos.append(np.array(xrefpos))
deltarefpos.append(np.array(yrefpos))
deltarefpos.append(np.array(zrefpos))

distance = np.cumsum(deltarefpos[0])
distance[len(distance)-1]
distance = np.cumsum(deltarefPos.getAxis(0))

# #################### #
# Motocross Field Data #
# #################### #
deltarefPos = data.ThreeData()
#deltarefPos.readData('../data/20131022_motocross_field/20131022-1812/20131207-1929/data/reference_delta_position.0.data', cov=False)
deltarefPos.readData('../data/20131022_motocross_field/20131022-1812/20131207-1929/data/odometry_delta_position.0.data', cov=False)
deltarefPos.eigenValues()
rot = quat.quaternion([0.8987, 0.0, 0.0, 0.4383])#Align reference trajectory
M = rot.toMatrix()
xrefpos = []
yrefpos = []
zrefpos = []
for i in range(0,len(deltarefPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xrefpos.append(vec[0])
    yrefpos.append(vec[1])
    zrefpos.append(vec[2])

deltarefpos=[]
deltarefpos.append(np.array(xrefpos))
deltarefpos.append(np.array(yrefpos))
deltarefpos.append(np.array(zrefpos))

distance = np.cumsum(deltarefpos[0])

# ############### #
# Test Track Data #
# ############### #
deltaodoPosDyn = data.ThreeData()
deltaodoPosDyn.readData('../data/20130415_motion_model_test_track/20131206-2159/data/odometry_delta_position.0.data', cov=True)
deltaodoPosDyn.eigenValues()

distance = np.cumsum(deltaodoPosDyn.getAxis(0))


