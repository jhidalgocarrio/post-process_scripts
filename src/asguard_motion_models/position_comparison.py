import datadisplay as data
from random import gauss
import csv, scipy
import math
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat


def deltasum(it):
    total = []
    for i in range(1,len(it)):
        total.append(it[i-1] + it[i])
    return total


#FrontEnd reference from Ground Truth
frontendreferencetest1139 = data.ThreeData()
frontendreferencetest1139.readData('data/multitest_spacehall/frontend_referencepose_position.1139.0.data', cov=True)
frontendreferencetest1139.eigenValues()

#FrontEnd position
frontendbodytest1139 = data.ThreeData()
frontendbodytest1139.readData('data/multitest_spacehall/frontend_poseout_position.1139.0.data', cov=True)
frontendbodytest1139.eigenValues()

#FrontEnd position
frontendbodytest1139under = data.ThreeData()
frontendbodytest1139under.readData('data/multitest_spacehall/frontend_poseout_position.1139.1.data', cov=True)
frontendbodytest1139under.eigenValues()

#FrontEnd velocity
frontendvelotest1139 = data.ThreeData()
frontendvelotest1139.readData('data/multitest_spacehall/frontend_poseout_velocity.1139.0.data', cov=True)
frontendvelotest1139.eigenValues()
frontendvelotest1139.plot_axis(1, 0, True, 1, True, [0,0,1])

#FrontEnd position
frontendposedynamic = data.ThreeData()
#frontendposedynamic.readData('data/normal_spacehall/weighting/frontend_poseout_position.testrack.dynamic_matrix.1144.0.data', cov=True)
#frontendposedynamic.readData('data/normal_spacehall/weighting/frontend_poseout_position.testrack.dynamic_matrix.1144.0.data', cov=True)
frontendposedynamic.readData('data/multitest_spacehall/weight1139/frontend_poseout_position.dynamic_matrix.1139.0.data', cov=True)
frontendposedynamic.eigenValues()

#FrontEnd position
frontendposenodynamic = data.ThreeData()
#frontendposenodynamic.readData('data/normal_spacehall/weighting/frontend_poseout_position.testrack.no_dynamic_matrix.1144.0.data', cov=True)
#frontendposenodynamic.readData('data/normal_spacehall/weighting/frontend_poseout_position.testrack.no_dynamic_matrix.1144.0.data', cov=True)
frontendposenodynamic.readData('data/multitest_spacehall/weight1139/frontend_poseout_position.no_dynamic_matrix.1139.0.data', cov=True)
frontendposenodynamic.eigenValues()

# Odometry Pose
odoPos = data.ThreeData()
odoPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131209-2022/data/odometry_position.0.data', cov=True)
odoPos.eigenValues()
odoOrient = data.QuaternionData()
odoOrient.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131119-1212/data/odometry_orientation.0.data', cov=True)
odoOrient.eigenValues()

# Odometry Pose
skidodoPos = data.ThreeData()
skidodoPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131209-1818/data/skid_odometry_position.0.data', cov=True)
skidodoPos.eigenValues()

# State Pose
statePos = data.ThreeData()
statePos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131129-2046/data/state_position.0.data', cov=True)
statePos.eigenValues()
stateOrient = data.QuaternionData()
stateOrient.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131128-1829/data/odometry_orientation.0.data', cov=True)
stateOrient.eigenValues()

# Delta State Pose
statedeltaPos = data.ThreeData()
statedeltaPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131129-2102/data/state_odo_delta_position.0.data', cov=True)
statedeltaPos.eigenValues()

# Delta Accelerometers Pose
accdeltaPos = data.ThreeData()
accdeltaPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131129-2102/data/state_acc_delta_position.0.data', cov=True)
accdeltaPos.eigenValues()

#GPS/Vicon reference from Ground Truth
refPos = data.ThreeData()
refPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131206-2344/data/reference_position.0.data', cov=False)
refPos.eigenValues()
refOrient = data.QuaternionData()
refOrient.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131128-1829/data/reference_orientation.0.data', cov=False)
refOrient.eigenValues()

#World to navigation
worldnavOrient = data.QuaternionData()
worldnavOrient.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131128-1829/data/world_2_nav_orient.0.data', cov=False)
worldnavOrient.eigenValues()


#GPS delta reference from Ground Truth
refdeltaPos = data.ThreeData()
refdeltaPos.readData('/home/jhidalgocarrio/esa-npi/dev/bundles/asguard/logs/20131119-1212/data/reference_delta_position.0.data', cov=False)
refdeltaPos.eigenValues()


#Loading  the script error_ellipse
finalpose= frontendbodytest1139.data[len(frontendbodytest1139.t)-1]
finalcov= frontendbodytest1139.cov[len(frontendbodytest1139.t)-1]
pointsreal = np.random.multivariate_normal(mean=(finalpose[0],finalpose[1]), cov=[[finalcov[0][0], finalcov[0][1]],[finalcov[1][0], finalcov[1][1]]], size=1000)
finalcovunder = frontendbodytest1139under.cov[len(frontendbodytest1139under.t)-1]
pointsunder = np.random.multivariate_normal(mean=(finalpose[0],finalpose[1]), cov=[[finalcovunder[0][0], finalcovunder[0][1]],[finalcovunder[1][0], finalcovunder[1][1]]], size=1000)

#Plotting velocity
plt.figure(1)
values = frontendvelotest1139.getAxis(0)
plt.plot(frontendvelotest1139.t, values,
        marker='.', label="Expected value with tactical-grade IMU", color=[0,0,1], lw=2)
plt.plot(frontendvelotest1139.t, frontendvelotest1139.getStdMax(0) , color=[0,0.5,1], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty with tactical-grade IMU')
plt.plot(frontendvelotest1139.t, frontendvelotest1139.getStdMin(0) , color=[0,0.5,1], linestyle='--', lw=2)
plt.ylabel(r'Velocity [$m/s$]',  fontsize=20)
plt.xlabel(r'Time [$s$]',  fontsize=20)
plt.grid(True)
plt.legend(prop={'size':20})
plt.show(block=False)

np.where(frontendvelotest1139.t > [7.8])
frontendbodytest1139.cov[708]
max(frontendbodytest1139.cov)


#Plotting motion model and ground truth comparison
plt.figure(1)
xposition = frontendbodytest1139.getAxis(0)
yposition = frontendbodytest1139.getAxis(1)
plt.plot(xposition, yposition,
        marker='.', label="Motion Model", color=[0,0,1], lw=2)
#xposition=frontendreferencetest1139.getAxis(0)
#yposition=frontendreferencetest1139.getAxis(1)
#plt.plot(xposition, yposition,
#        marker='D', label="Ground Truth", color=[0,0.5,0.5], alpha=0.5, lw=5)

# Plot the raw points...
#x, y = points.T
#plt.plot(x, y, 'ro')

# Plot a transparent 3 standard deviation covariance ellipse
plot_point_cov(pointsreal, nstd=1.1, alpha=0.5, color='green')
plot_point_cov(pointsunder, nstd=0.33, alpha=0.5, color='red')
plot_point_cov(pointsunder, nstd=2.8, alpha=0.5, color='red')
plt.ylabel(r'Velocity [$m/s$]',  fontsize=14)
plt.xlabel(r'Time [$s$]',  fontsize=14)
plt.grid(True)
plt.legend(prop={'size':15})
plt.show(block=False)

#Customize plotting
plt.figure(1)
xposition = frontendposedynamic.getAxis(0)
yposition = frontendposedynamic.getAxis(1)
plt.plot(xposition, yposition,
        marker='*', label="Trajectory with wheel compensation", color=[0,0,1], lw=2)
xposition = frontendposenodynamic.getAxis(0)
yposition = frontendposenodynamic.getAxis(1)
plt.plot(xposition, yposition,
        marker='.', label="Trajectory without wheel compensation", color=[0,1,0], lw=2)
xposition = frontendreferencetest1139.getAxis(0)
yposition = frontendreferencetest1139.getAxis(1)
plt.plot(xposition, yposition,
        marker='.', label="Ground Truth", linestyle='--', color=[1,0,0], lw=2)
plt.ylabel(r' Position [$m$]')
plt.xlabel(r' Position [$m$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/position_dynamic_vs_no_dynamic_vicon.png')


#Plotting GPS and Odometry values
plt.figure(1)
xposition = refPos.getAxis(0)
yposition = refPos.getAxis(1)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.plot(xposition, yposition, marker='D', label="GPS Ground Truth", color=[0.5,0,0], alpha=0.5, lw=2)
plt.annotate(r'Starting point', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=18,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End point', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(-10, -30), textcoords='offset points', fontsize=18,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
xposition = odoPos2010.getAxis(0)
yposition = odoPos2010.getAxis(1)
plt.plot(xposition, yposition, marker='.', label="Asguard Odometry", color=[0,0,1], lw=2)
plt.ylabel(r'Position [$m$]')
plt.xlabel(r'Position [$m$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

#Odometry and Skid Odometry values (X-Y Axes)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(1)
xposition = odoPos.getAxis(0)
yposition = odoPos.getAxis(1)
plt.plot(xposition, yposition, marker='*', linestyle='--', label="Full Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-20, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(+20, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

xposition = odoPosDyn.getAxis(0)
yposition = odoPosDyn.getAxis(1)
plt.plot(xposition, yposition, marker='*', linestyle='-.', label="Full Odometry + Support Polygon", color=[0.0,0.8,0], alpha=0.5, lw=2)

xposition = skidodoPos.getAxis(0)
yposition = skidodoPos.getAxis(1)
plt.plot(xposition, yposition, marker='.', linestyle='.-', label="Conventional Odometry", color=[0,0.5,1], lw=2)
plt.xlabel(r' Position in X [$m$]', fontsize=24)
plt.ylabel(r' Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/test_track_position_x_y.png')


#Odometry and Skid Odometry values (X-Z Axis)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(2)
xposition = odoPos.getAxis(0)
zposition = odoPos.getAxis(2)
plt.plot(xposition, zposition, marker='*', linestyle='--', label="Full Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-20, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(+20, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

xposition = odoPosDyn.getAxis(0)
zposition = odoPosDyn.getAxis(2)
plt.plot(xposition, zposition, marker='*', linestyle='-.', label="Full Odometry + Support Polygon", color=[0.0,0.8,0], alpha=0.5, lw=2)

xposition = skidodoPos.getAxis(0)
zposition = skidodoPos.getAxis(2)
plt.plot(xposition, zposition, marker='.', linestyle='.-', label="Conventional Odometry", color=[0,0.5,1], lw=2)
plt.xlabel(r' Position in X [$m$]', fontsize=24)
plt.ylabel(r' Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/test_track_position_x_z.png')



#3D Plotting Odometry and Skid odometry values
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.gca(projection='3d')

xposition = odoPosDyn.getAxis(0)
yposition = odoPosDyn.getAxis(1)
zposition = odoPosDyn.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='.', label="Full Odometry + Support Polygon", color=[0.7,0.4,0.0], lw=2)

xposition = skidodoPos.getAxis(0)
yposition = skidodoPos.getAxis(1)
zposition = skidodoPos.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='.', label="Conventional Odometry", color=[0,0.0,1], lw=2)

ax.zaxis.set_major_locator(LinearLocator(10))
ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

ax.grid(True)
ax.legend(prop={'size':25})
plt.ylabel(r'Position [$m$]')
plt.xlabel(r'Position [$s$]')
ax.set_xlim([-3,60])
ax.set_ylim([-10,10])
ax.set_zlim([0,5])
plt.show(block=False)


#Odometry , Skid Odometry and GPS values(X-Y Axis Sand Field)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(1)
rot = quat.quaternion([0.819, -0.014, 0.01001, -0.5735])
M = rot.toMatrix()
xposition = []
yposition = []

for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])

plt.plot(xposition, yposition, marker='D', linestyle='--', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-10, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
xposition = odoPos.getAxis(0)
yposition = odoPos.getAxis(1)
plt.plot(xposition, yposition, marker='*', linestyle='--', label="Full Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

xposition = odoPosDyn.getAxis(0)
yposition = odoPosDyn.getAxis(1)
plt.plot(xposition, yposition, marker='*', linestyle='-.', label="Full Odometry + Support Polygon", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)
yposition = skidodoPos.getAxis(1)
plt.plot(xposition, yposition, marker='.', label="Conventional Odometry", color=[0,0.5,1], lw=2)
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/sandfield_position_x_y.png')


#Odometry , Skid Odometry and GPS values(X-Z Axis Sand Field)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(2)
rot = quat.quaternion([0.819, -0.014, 0.01001, -0.5735])
M = rot.toMatrix()
xposition = []
zposition = []

for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    zposition.append(vec[2])

plt.plot(xposition, zposition, marker='D', linestyle='--', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)
plt.scatter(xposition[0], zposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], zposition[len(zposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], zposition[0]), xycoords='data',
                                xytext=(-70, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[len(xposition)-1], zposition[len(zposition)-1]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
xposition = odoPos.getAxis(0)
zposition = odoPos.getAxis(2)
plt.plot(xposition, zposition, marker='*', linestyle='--', label="Full Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

xposition = odoPosDyn.getAxis(0)
zposition = odoPosDyn.getAxis(2)
plt.plot(xposition, zposition, marker='*', linestyle='-.', label="Full Odometry + Support Polygon", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)
zposition = skidodoPos.getAxis(2)
plt.plot(xposition, zposition, marker='.', label="Conventional Odometry", color=[0,0.5,1], lw=2)
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/sandfield_position_x_z.png')


#Odometry , Skid Odometry and GPS values(X-Y Axis Motocross Field)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(1)
#rot = quat.quaternion([0.898, 0.0, 0.0, 0.4383]) #only heading
rot = quat.quaternion([0.8984, 0.0038, 0.0078, 0.4383])
M = rot.toMatrix()
xposition = []
yposition = []

for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])

plt.plot(xposition, yposition, marker='D', linestyle='--', label="RTK GPS", color=[0.5,0,0], alpha=0.5, lw=2)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-10, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
xposition = odoPos.getAxis(0)
yposition = odoPos.getAxis(1)
plt.plot(xposition, yposition, marker='*', linestyle='--', label="Full Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

xposition = odoPosDyn.getAxis(0)
yposition = odoPosDyn.getAxis(1)
plt.plot(xposition, yposition, marker='*', linestyle='-.', label="Full Odometry + Support Polygon", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)
yposition = skidodoPos.getAxis(1)
plt.plot(xposition, yposition, marker='.', label="Conventional Odometry", color=[0,0.5,1], lw=2)
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(loc=4, prop={'size':25})
plt.show(block=False)
savefig('figures/motocross_position_x_y.png')


#Odometry , Skid Odometry and GPS values(X-Z Axis Motocross Field)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(2)
rot = quat.quaternion([0.8984, 0.0038, 0.0078, 0.4383])
M = rot.toMatrix()
xposition = []
zposition = []

for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    zposition.append(vec[2])

plt.plot(xposition, zposition, marker='D', linestyle='--', label="GPS", color=[0.5,0,0], alpha=0.5, lw=2)
plt.scatter(xposition[0], zposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], zposition[len(zposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], zposition[0]), xycoords='data',
                                xytext=(-70, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[len(xposition)-1], zposition[len(zposition)-1]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
xposition = odoPos.getAxis(0)
zposition = odoPos.getAxis(2)
plt.plot(xposition, zposition, marker='*', linestyle='--', label="Full Odometry", color=[0.3,0.2,0.4], alpha=0.5, lw=2)

xposition = odoPosDyn.getAxis(0)
zposition = odoPosDyn.getAxis(2)
plt.plot(xposition, zposition, marker='*', linestyle='-.', label="Full Odometry + Support Polygon", color=[0.0,0.8,0], alpha=0.5, lw=2)


xposition = skidodoPos.getAxis(0)
zposition = skidodoPos.getAxis(2)
plt.plot(xposition, zposition, marker='.', label="Conventional Odometry", color=[0,0.5,1], lw=2)
plt.xlabel(r'Position in X [$m$]', fontsize=24)
plt.ylabel(r'Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/motocross_position_x_z.png')

#3D Plotting GPS values
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.gca(projection='3d')

rot = quat.quaternion([0.898794046299167, 0.00, 0.00, 0.4383711467890774])
M = rot.toMatrix()
xposition = []
yposition = []
zposition = []

for i in range(0,len(refPos.data)):
    x = refPos.data[i][0]
    y = refPos.data[i][1]
    z = refPos.data[i][2]
    vec = dot(M,[x,y,z])
    xposition.append(vec[0])
    yposition.append(vec[1])
    zposition.append(vec[2])

ax.plot(xposition, yposition, zposition, marker='D', label="GPS Ground Truth", color=[0.5,0,0], alpha=0.5, lw=2)

xposition = odoPos.getAxis(0)
yposition = odoPos.getAxis(1)
zposition = odoPos.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='.', label="Odometry", color=[0,0.4,1], lw=2)

xposition = skidodoPos.getAxis(0)
yposition = skidodoPos.getAxis(1)
zposition = skidodoPos.getAxis(2)
ax.plot(xposition, yposition, zposition, marker='.', label="Skid Odometry", color=[0,0.5,1], lw=2)

ax.grid(True)
ax.legend(prop={'size':25})
plt.ylabel(r'Position [$m$]')
plt.xlabel(r'Position [$s$]')
ax.set_xlim([0,100])
ax.set_ylim([0,100])
ax.set_zlim([0,10])
plt.show(block=False)

#Plotting Orientation values
plt.figure(1)
time = odoOrient.t
euler = []
euler.append(odoOrient.getEuler(2))# Roll
euler.append(odoOrient.getEuler(1))# Pitch
euler.append(odoOrient.getEuler(0))# Yaw

euler[0][:] = [x * 180.00/math.pi for x in euler[0] ]#convert to degrees
euler[1][:] = [x * 180.00/math.pi for x in euler[1] ]#convert to degrees
euler[2][:] = [x * 180.00/math.pi for x in euler[2] ]#convert to degrees

plt.plot(time, euler[0], marker='.', label="Roll", color=[1.0,0,0], alpha=0.5, lw=2)
plt.plot(time, euler[1], marker='.', label="Pitch", color=[0,1.0,0], alpha=0.5, lw=2)
plt.plot(time, euler[2], marker='.', label="Yaw", color=[0,0,1.0], alpha=0.5, lw=2)
plt.xlabel(r'Time [$s$]')
plt.ylabel(r'Angle [${}^\circ$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


#Plotting Vicon and Odometry values
plt.figure(2)
xposition = refPos.getAxis(0)
yposition = refPos.getAxis(1)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.plot(xposition, yposition, marker='D', label="Ground Truth", color=[0.5,0,0], alpha=0.5, lw=2)
plt.annotate(r'Starting point', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(+10, +30), textcoords='offset points', fontsize=18,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End point', xy=(xposition[len(xposition)-1], yposition[len(yposition)-1]), xycoords='data',
                                xytext=(-10, -30), textcoords='offset points', fontsize=18,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
xposition = statePos.getAxis(0)
yposition = statePos.getAxis(1)
plt.plot(xposition, yposition, marker='.', label="State Optimize", color=[0,0,1], lw=2)
plt.ylabel(r'Position [$m$]')
plt.xlabel(r'Position [$m$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

#Plotting state position versus time
plt.figure(1)
values = statePos.getAxis(0)
plt.plot(statePos.t, values,
        marker='.', label="Motion Model X-axis", color=[0,1,0], lw=2)


# Delta pose comparison
plt.figure(4)
values = statedeltaPos.getAxis(0)
plt.plot(statedeltaPos.t, values,
        marker='.', label="Motion Model X-axis", color=[0,1,0], lw=2)
plt.plot(statedeltaPos.t, statedeltaPos.getStdMax(0) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 1\sigma$ Motion Model uncertainty')
plt.plot(statedeltaPos.t, statedeltaPos.getStdMin(0) , color=[0,0,0], linestyle='--', lw=2)
values=accdeltaPos.getAxis(0)
plt.plot(accdeltaPos.t, values,
        marker='D', label="Accelerometers X-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.plot(accdeltaPos.t, accdeltaPos.getStdMax(0) , color=[0.5,0.5,0.5], linestyle='--', lw=2, label=r'$\pm 1\sigma$ Motion Model uncertainty')
plt.plot(accdeltaPos.t, accdeltaPos.getStdMin(0) , color=[0.5,0.5,0.5], linestyle='--', lw=2)
plt.ylabel(r'Delta Position [$m$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)

# Comparison State and Cumsum delta state
plt.figure(3)
xposition = statePos.getAxis(0)
yposition = statePos.getAxis(1)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[len(xposition)-1], yposition[len(yposition)-1], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.plot(xposition, yposition, marker='D', label="State Optimize", color=[0.5,0,0], alpha=0.5, lw=2)

values = statedeltaPos.getAxis(0)
xcumpos = xposition[0] + np.cumsum(values)
values = statedeltaPos.getAxis(1)
ycumpos = yposition[0] + np.cumsum(values)
plt.plot(xcumpos, ycumpos, marker='.', label="Cumulative Sum Delta State Optimize", color=[1,0,0], lw=2)

values = accdeltaPos.getAxis(0)
xcumpos = xposition[0] + np.cumsum(values)
values = accdeltaPos.getAxis(1)
ycumpos = yposition[0] + np.cumsum(values)
#values = [x * 1 * 1 for x in accx ]
#xcumpos = xposition[0] + np.cumsum(values)
#values = [y * 1 * 1 for y in accy ]
#ycumpos = yposition[0] + np.cumsum(values)
plt.plot(xcumpos, ycumpos, marker='.', label="Cumulative Sum Delta Acceleration", color=[0.2,0,0], lw=2)
plt.ylabel(r'Position [$m$]')
plt.xlabel(r'Position [$m$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)


# Full Odometry values with inclinometers in attitude correction(X-Y Axes)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(1)

xposition = odoPosDynInc.getAxis(0)
yposition = odoPosDynInc.getAxis(1)
plt.plot(xposition, yposition, marker='.', linestyle='.-', label="Full Odometry + Support Polygon + Inc", color=[0.5,0,1], lw=2)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0,0.5,0.5], alpha=0.5, lw=20)
plt.scatter(xposition[0], yposition[0], marker='D', color=[0.5,0,0.5], alpha=0.5, lw=20)
plt.annotate(r'Start', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(-20, -30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
plt.annotate(r'End', xy=(xposition[0], yposition[0]), xycoords='data',
                                xytext=(+20, +30), textcoords='offset points', fontsize=22,
                                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))


xposition = odoPosDyn.getAxis(0)
yposition = odoPosDyn.getAxis(1)
plt.plot(xposition, yposition, marker='*', linestyle='-.', label="Full Odometry + Support Polygon", color=[0.0,0.8,0], alpha=0.5, lw=2)
plt.xlabel(r' Position in X [$m$]', fontsize=24)
plt.ylabel(r' Position in Y [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/test_track_position_with_inc_x_y.png')


# Full Odometry values with inclinometers in attitude correction(X-Z Axes)
matplotlib.rcParams.update({'font.size': 25})
plt.figure(2)

xposition = odoPosDynInc.getAxis(0)
zposition = odoPosDynInc.getAxis(2)
plt.plot(xposition, zposition, marker='.', linestyle='.-', label="Conventional Odometry + Support Polygon + Inc", color=[0.5,0,1], lw=2)

xposition = odoPosDyn.getAxis(0)
zposition = odoPosDyn.getAxis(2)
plt.plot(xposition, zposition, marker='*', linestyle='-.', label="Full Odometry + Support Polygon", color=[0.0,0.8,0], alpha=0.5, lw=2)
plt.xlabel(r' Position in X [$m$]', fontsize=24)
plt.ylabel(r' Position in Z [$m$]', fontsize=24)
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/test_track_position_with_inc_x_z.png')

