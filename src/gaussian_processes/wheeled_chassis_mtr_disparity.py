import math
import numpy as np
from sklearn.gaussian_process import GaussianProcess
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib import cm


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
vmax = 0.10
vmin = -0.10
xi = np.linspace(vmin, vmax, npts)
yi = np.linspace(vmin, vmax, npts)


################################################
#xx, yy = np.meshgrid(xi, yi, sparse=True)
xx, yy = np.meshgrid(xi, yi)

# Movement Function #
rot =  (yy - xx)/0.3
v = (xx+yy)/2.0
zx = v*np.cos(rot) # velocity in x
zy = v*np.sin(rot) # velocity in y
z = zx + zy


fig = plt.figure(11)
ax = fig.gca(projection='3d')
ax.plot_surface(xx, yy, z, rstride=4, cstride=4, alpha=0.3)
cset = ax.contour(xx, yy, z, zdir='z', cmap=plt.cm.coolwarm,
        vmax=abs(z).max(), vmin=-abs(z).max(), linewidth=4.0)
cset = ax.contour(xx, yy, z, zdir='x', cmap=cm.coolwarm)
cset = ax.contour(xx, yy, z, zdir='y', cmap=cm.coolwarm)

ax.set_xlabel('X')
ax.set_xlim(min(xi), max(xi))
ax.set_ylabel('Y')
ax.set_ylim(min(yi), max(yi))
ax.set_zlabel('Z')

plt.show(block=False)

################################################
fig = plt.figure(3)
CS = plt.contour(xx,yy,z,18)
cbar = plt.colorbar(CS)
cbar.ax.set_ylabel('mean value')
# Add the contour line levels to the colorbar
cbar.add_lines(CS)
plt.show(block=False)


