#! /usr/bin/env python
# -*- coding:utf-8 -*-
# by javi 2017-02-21 14:44:20
import csv, scipy
import math
from pylab import *
import numpy as np
from matplotlib import pyplot as plt
from math import sqrt
matplotlib.style.use('classic') #in matplotlib >= 1.5.1

# Gaussian processes residual boundaries
gp_min_boundary = 0.00
gp_max_boundary = 0.015

# Images frame pe
image_frame_min = 0.4
image_frame_max = 2.8

def linear_frame(constant, t, min_value):
    return constant * t + min_value;

def quadratic_frame(constant, t, min_value):
    return constant * pow(t, 2) + min_value;

def exponential_frame(constant, t, min_value):
    return -exp(-constant * pow(t, 2)) + 1.0 + min_value;

def exponential_ratio(constant, t, min_value):
    return exp(constant * pow(t, 2)) - 1.0 + min_value;


################################
# Residual vs image frame period
################################
matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)

residual_1 = np.arange(gp_min_boundary, gp_max_boundary, 0.0001)
residual_2 = np.arange(gp_min_boundary, gp_max_boundary, 0.001)

# Quadratic
eq_constant = (image_frame_min - image_frame_max) / pow(gp_max_boundary - gp_min_boundary, 2);
ax.plot(residual_1, quadratic_frame(eq_constant, residual_1, image_frame_max),
        linestyle='-', marker=".", lw=4, alpha=1.0, color=[0, 0, 0.0],
        label='Quadratic')

#ax.scatter(residual_2, quadratic_frame(eq_constant, residual_2, image_frame_max),
#        marker='o', color=[0.0,0.0,1.0], s=100, alpha=0.5, label='Quadratic')

# Exponential
ax.plot(residual_1, exponential_frame(eq_constant, residual_1, image_frame_max),
        linestyle='--', marker="+", lw=4, alpha=1.0, color=[0, 0, 0.0],
        label='Exponential')

#ax.scatter(residual_2, exponential_frame(eq_constant, residual_2, image_frame_max),
#        marker='*', color=[0.0,1.0,0.0], s=100, alpha=0.5, label='Exponential')

# Linear
eq_constant = (image_frame_min - image_frame_max) / (gp_max_boundary - gp_min_boundary);
ax.plot(residual_1, linear_frame(eq_constant, residual_1, image_frame_max),
        linestyle='-.', marker="x", lw=4, alpha=1.0, color=[0.0, 0, 0.0],
        label='Linear')

#ax.scatter(residual_2, linear_frame(eq_constant, residual_2, image_frame_max),
#        marker='D', color=[1.0,0.0,0.0], s=100, alpha=0.5, label='Linear')

ax.set_xlim(gp_min_boundary, gp_max_boundary)
ax.set_ylim(image_frame_min, image_frame_max+0.02)

plt.xlabel(r'Odometry error [$m/s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Visual odometry period', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':35})
plt.show(block=True)
fig.savefig("function_plot_frame_adaptive_slam.png", dpi=fig.dpi)

#########################################
# Residual vs image frame period (latex )
#########################################
matplotlib.rcParams.update({'font.size': 25, 'font.weight': 'bold',
    'text.usetex': True, 'text.latex.preamble': r'\usepackage{amsmath}'})
fig = plt.figure(1, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)

residual_1 = np.arange(gp_min_boundary, gp_max_boundary, 0.0001)
residual_2 = np.arange(gp_min_boundary, gp_max_boundary, 0.001)

# Quadratic
eq_constant = (image_frame_min - image_frame_max) / pow(gp_max_boundary - gp_min_boundary, 2);
ax.plot(residual_1, quadratic_frame(eq_constant, residual_1, image_frame_max),
        linestyle='--', lw=2, alpha=1.0, color=[0, 0, 0.0])

ax.scatter(residual_2, quadratic_frame(eq_constant, residual_2, image_frame_max),
        marker='o', color=[0.0,0.0,1.0], s=100, alpha=0.5, label='Quadratic')

# Exponential
ax.plot(residual_1, exponential_frame(eq_constant, residual_1, image_frame_max),
        linestyle='--', lw=2, alpha=1.0, color=[0, 0, 0.0])

ax.scatter(residual_2, exponential_frame(eq_constant, residual_2, image_frame_max),
        marker='*', color=[0.0,1.0,0.0], s=100, alpha=0.5, label='Exponential')

# Linear
eq_constant = (image_frame_min - image_frame_max) / (gp_max_boundary - gp_min_boundary);
ax.plot(residual_1, linear_frame(eq_constant, residual_1, image_frame_max),
        linestyle='--', lw=2, alpha=1.0, color=[0.0, 0, 0.0])

ax.scatter(residual_2, linear_frame(eq_constant, residual_2, image_frame_max),
        marker='D', color=[1.0,0.0,0.0], s=100, alpha=0.5, label='Linear')

ax.set_xlim(gp_min_boundary, gp_max_boundary+0.001)
ax.set_ylim(image_frame_min, image_frame_max+0.2)


ax.set_xticklabels(["$\underline{\gamma}$", "", "", "", "", "", "", "", "$\overline{\gamma}$"])
ax.set_yticklabels(["", r"$\underline{\rho}$", "", "", "", "",r"$\overline{\rho}$"])

plt.xlabel(r'Odometry error [$m/s$]', fontsize=45, fontweight='bold')
plt.ylabel(r'Visual odometry period', fontsize=45, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':35})
plt.show(block=True)

################################
# Matches ratio
################################
matches_ratio_min = 0.00 #0.3
matches_ratio_max = 1.00 #0.75

################################
# Residual vs image frame period
################################
matplotlib.rcParams.update({'font.size': 40, 'font.weight': 'bold'})
fig = plt.figure(2, figsize=(28, 16), dpi=120, facecolor='w', edgecolor='k')
ax = fig.add_subplot(111)

residual_1 = np.arange(gp_min_boundary, gp_max_boundary, 0.0001)
residual_2 = np.arange(gp_min_boundary, gp_max_boundary, 0.001)


# Quadratic
eq_constant = (matches_ratio_max - matches_ratio_min) / pow(gp_max_boundary - gp_min_boundary, 2)
ax.plot(residual_1, quadratic_frame(eq_constant, residual_1, matches_ratio_min),
        linestyle='-', marker=".", lw=4, alpha=1.0, color=[0, 0, 0.0],
        label='Quadratic')

#ax.scatter(residual_2, quadratic_frame(eq_constant, residual_2, matches_ratio_min),
#        marker='o', color=[0.0,0.0,1.0], s=100, alpha=0.5, label='Quadratic')

# Exponential
ax.plot(residual_1, exponential_ratio(eq_constant, residual_1, matches_ratio_min),
        linestyle='--', marker="+", lw=4, alpha=1.0, color=[0, 0, 0.0],
        label='Exponential')

#ax.scatter(residual_2, exponential_ratio(eq_constant, residual_2, matches_ratio_min),
#        marker='*', color=[0.0,1.0,0.0], s=100, alpha=0.5, label='Exponential')

# Linear
eq_constant = (matches_ratio_max - matches_ratio_min) / (gp_max_boundary - gp_min_boundary);
ax.plot(residual_1, linear_frame(eq_constant, residual_1, matches_ratio_min),
        linestyle='-.', marker="x", lw=4, alpha=1.0, color=[0.0, 0, 0.0],
        label='Linear')

#ax.scatter(residual_2, linear_frame(eq_constant, residual_2, matches_ratio_min),
#        marker='D', color=[1.0,0.0,0.0], s=100, alpha=0.5, label='Linear')

ax.set_xlim(gp_min_boundary, gp_max_boundary)
ax.set_ylim(matches_ratio_min, matches_ratio_max)
#ax.get_xaxis().set_visible(False)

plt.xlabel(r'Odometry error [$m/s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Feature matching ratio', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=2, prop={'size':35})
plt.show(block=True)
fig.savefig("function_plot_ratio_adaptive_slam.png", dpi=fig.dpi)

