import math
from pylab import *
import numpy as np
from sklearn.gaussian_process import GaussianProcess
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib import cm

# It represent the basic idea to Model to Reality (MTR) disparity
def f(dw,l=0.20):
    """Simple Two wheel motion model."""
    """ dw[0] is left wheel """
    """ dw[1] is right wheel """
    rot = -(dw[1] - dw[0])/l
    v = (dw[0]+dw[1])/2.0
    #print rot
    #print v
    return np.array([v*math.cos(rot),v*math.sin(rot)])


# define grid.
npts=100
vmax = 0.10 # max velocity in meters per second
vmin = -0.10 # min velocity in meters per second (sign is direction)
xi = np.linspace(vmin, vmax, npts)
yi = np.linspace(vmin, vmax, npts)


################################################
#xx, yy = np.meshgrid(xi, yi, sparse=True)
xx, yy = np.meshgrid(xi, yi)

# Movement Function #
rot =  (yy - xx)/0.3 # rotational velocity (heading rate)
v = (0.5 * xx + 0.5 * yy) # linear velocity assuming both wheels have the same weight.
zx = v*np.cos(rot) # velocity in x
zy = v*np.sin(rot) # velocity in y
z = zx + zy # resulting body velocity
#z = sqrt(zx*zx + zy*zy) # resulting body velocity

################################################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.plot_surface(xx, yy, z, rstride=4, cstride=4, alpha=0.3)
cset = ax.contour(xx, yy, z, zdir='z', cmap=plt.cm.coolwarm,
        vmax=abs(z).max(), vmin=-abs(z).max(), linewidth=4.0)
cset = ax.contour(xx, yy, z, zdir='x', cmap=cm.coolwarm)
cset = ax.contour(xx, yy, z, zdir='y', cmap=cm.coolwarm)
ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_xlim(min(xi), max(xi))
ax.set_ylabel('Y')
ax.set_ylim(min(yi), max(yi))
ax.set_zlabel('Z')

plt.show(block=False)

################################################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(2, figsize=(10,10))
CS = plt.contour(xx,yy,z,18)
cbar = plt.colorbar(CS)
cbar.ax.set_ylabel('mean value')
# Add the contour line levels to the colorbar
cbar.add_lines(CS)
plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show(block=False)

################################################
from scipy import stats
noise=stats.distributions.norm.rvs(0, 0.003, size=z.shape)
z_noise = z + noise
################################################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.plot_surface(xx, yy, z_noise, rstride=4, cstride=4, alpha=0.3)
cset = ax.contour(xx, yy, z_noise, zdir='z', cmap=plt.cm.coolwarm,
        vmax=abs(z_noise).max(), vmin=-abs(z_noise).max(), linewidth=4.0)
cset = ax.contour(xx, yy, z_noise, zdir='x', cmap=cm.coolwarm)
cset = ax.contour(xx, yy, z_noise, zdir='y', cmap=cm.coolwarm)
ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_xlim(min(xi), max(xi))
ax.set_ylabel('Y')
ax.set_ylim(min(yi), max(yi))
ax.set_zlabel('Z')

plt.show(block=False)

################################################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.plot_surface(xx, yy, noise, rstride=4, cstride=4, alpha=0.3)
cset = ax.contour(xx, yy, noise, zdir='z', cmap=plt.cm.coolwarm,
        vmax=abs(noise).max(), vmin=-abs(noise).max(), linewidth=4.0)
cset = ax.contour(xx, yy, noise, zdir='x', cmap=cm.coolwarm)
cset = ax.contour(xx, yy, noise, zdir='y', cmap=cm.coolwarm)
ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_xlim(min(xi), max(xi))
ax.set_ylabel('Y')
ax.set_ylim(min(yi), max(yi))
ax.set_zlabel('Z')
ax.set_zlim(0.12, -0.12)

plt.show(block=False)

