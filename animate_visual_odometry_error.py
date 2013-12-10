import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from matplotlib.patches import Ellipse

fig = plt.figure(figsize=(8,6), dpi=150)
ax = fig.add_subplot(111)
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.spines['bottom'].set_position(('data',0))
ax.yaxis.set_ticks_position('left')
ax.spines['left'].set_position(('data',0))
ax.grid()
xdata, ydata = [], []

line, = plt.plot([], [], linestyle='none', marker='o', color='r')

def data_gen():
    i = 0
    while i< 100:
        x0 = random.random()
        y0 = random.random()
        i = i + 1
        cov=np.array([[0.02, 0.01],
                [0.01, 0.02]])
        yield x0, y0, cov

def run(data):
    x0,y0,cov = data
    nstd=1
    xdata.append(x0)
    ydata.append(y0)
    line.set_data(xdata, ydata)

    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2 * nstd * np.sqrt(vals)
    ellip = Ellipse([x0,y0], width, height, theta, alpha=0.5, color='green')
    artirst = ax.add_artist(ellip)

    return line,artist,

ani = animation.FuncAnimation(fig, run, data_gen, blit=True, interval=0.5, repeat=False)
plt.show()

