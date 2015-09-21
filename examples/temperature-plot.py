#!/usr/bin/env python
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap as lscm
from matplotlib.colorbar import ColorbarBase as cbb
from matplotlib.font_manager import FontProperties
import matplotlib.gridspec as gridspec
from numpy import *

#define a pulse
def pulse(x, mid, width_top, width_bottom):
    if abs(mid-x)<.5*width_top:
        return 1.0;
    elif abs(mid-x)<.5*width_bottom:
        if (x>mid):
            return 1.-2*(x-(mid+.5*width_top))/(width_bottom-width_top);
        else:
            return 1.-2*((mid-.5*width_top)-x)/(width_bottom-width_top);
    else:
        return 0.;

#generate pulse train
#t = arange(0,1,.005)
t = linspace(0,1,9)
pr = array([pulse(i,.75,.245,.75) for i in t])
pg = array([pulse(i,.5,.245,.75) for i in t])
pb = array([pulse(i,.25,.245,.75) for i in t])
colors = [(pr[i],pg[i],pb[i]) for i in range(len(pr))]

print 255*array(colors)

#set up two subplots with height ratio
gs = gridspec.GridSpec(2, 1,height_ratios=[4,1])

#set global font family
plt.rcParams['font.family'] = 'cursive'

#get figure handle
fig = plt.figure()

#main plot
ax1 = fig.add_subplot(gs[0],frameon=False)
ax1.plot(t,pb,color='b',lw=2)
ax1.plot(t,pg,color='g',lw=2)
ax1.plot(t,pr,color='r',lw=2)
title = 'temperature color map from RGB pulse train'
plt.title(title,fontsize=20)
labels = ['blue','green','red']
legfont = FontProperties(size=10)
ax1.legend(labels,loc='lower left',title='',prop=legfont)
plt.xlabel('input')
plt.ylabel('output')
ax1.grid(False)

#color bar
ax2 = fig.add_subplot(gs[1])
cm = lscm.from_list('temp',colors)
cbb(ax2,cm,orientation='horizontal')

#save it out
fig.savefig('plot-pulses.png',transparent=True)
