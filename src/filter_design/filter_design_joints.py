#!/usr/bin/env python

import csv, scipy, numpy, pylab
import scipy.fftpack
from pylab import *
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.signal import filter_design as fd
import scipy.signal as sig

#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140605_pink_odometry/20140605-1731/data/left_passive_position.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140522-2014_new_transformer/20140529-1903/left_passive_position.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140603_passive_joints/20140604-1134/left_passive_position_localization_frontend.data', 'rb'), delimiter=' ', quotechar='|')
spamReader = csv.reader(open('/home/jhidalgocarrio/exoter/experiments/20140818_passive_joints/20140818-1715/left_passive_position.data', 'rb'), delimiter=' ', quotechar='|')

time=[]
joint=[]

for row in spamReader:
    #print row
    time.append(float(row[0])/1000000)
    joint.append(float(row[1]))

delta = []
for i in range(0,len(time)-1):
    #print time[i]
    t = float(time[i+1]) - float(time[i])
    delta.append(t)

delta_t = mean(delta)
sample_rate = 1/delta_t
t = delta_t * r_[0:len(time)]

# The Nyquist rate of the signal.
nyq_rate = sample_rate / 2.

# The cutoff frequency of the filter (in Hz)
cutoff_hz = 1.0

# Length of the filter (number of coefficients, i.e. the filter order + 1)
numtaps = 26

###################
### IIR FILTER  ###
###################

# Specification for our filter
Wp = cutoff_hz/nyq_rate # Cutoff frequency
Ws = (cutoff_hz+1.5)/nyq_rate   # Stop frequency
Rp = 1     # passband maximum loss (gpass)
As = 42      # stoppand min attenuation (gstop)

Filters = {'ellip' : (), 'cheby2' : (), 'butter' : (), 'cheby1' : (),  'bessel' : ()}

# The ellip and cheby2 filter design
Filters['ellip'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='ellip')
Filters['cheby2'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='cheby2')
Filters['butter'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='butter')
Filters['cheby1'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='cheby1')

# The bessel max order of 8 for this cutoff, can't use
# iirdesign have to use iirfilter.
Filters['bessel'] = fd.iirfilter(8, Wp, btype='lowpass', ftype='bessel')


## Pass the signal though the filter
jointfilter = {'ellip' : (), 'cheby2' : (), 'butter' : (), 'cheby1' : (),  'bessel' : ()}
jointfilter['ellip'] = sig.lfilter(Filters['ellip'][0], Filters['ellip'][1], joint)
jointfilter['cheby2'] = sig.lfilter(Filters['cheby2'][0], Filters['cheby2'][1], joint)
jointfilter['butter'] = sig.lfilter(Filters['butter'][0], Filters['butter'][1], joint)
jointfilter['cheby1'] = sig.lfilter(Filters['cheby1'][0], Filters['cheby1'][1], joint)
jointfilter['bessel'] = sig.lfilter(Filters['bessel'][0], Filters['bessel'][1], joint)

b=Filters['bessel'][0] #feedforward
a=Filters['bessel'][1] #feedback

###################
### FIR FILTER  ###
###################

Wp = cutoff_hz/nyq_rate

# Use firwin to create a lowpass FIR filter
Filters['fir_hamming'] = sig.firwin(numtaps, cutoff = Wp, window = "hamming")

# Use firwin to create a lowpass FIR filter. The 1.0 is the a coefficient for FIR
jointfilter['fir_hamming'] = sig.lfilter(Filters['fir_hamming'], 1.0, joint)



#################
### GRAPHICS  ###
#################
plt.figure(4)
pylab.plot(t,joint, '-o', label="Passive Joint")
pylab.plot(t,jointfilter['ellip'], '-o', label="ellip filter")
pylab.plot(t,jointfilter['cheby2'], '-o', label="cheby2 filter")
pylab.plot(t,jointfilter['butter'], '-o', label="butter filter")
pylab.plot(t,jointfilter['cheby1'], '-o', label="cheby1 filter")
pylab.plot(t,jointfilter['bessel'], '-o', label="bessel filter")
pylab.plot(t,jointfilter['fir_hamming'], '-o', label="FIR filter")
plt.legend(prop={'size':25})
grid(True)
plt.show(block=False)


plt.figure(5)

jointdeg = [x * 180.00/math.pi for x in joint]
jointfilterdeg = {'ellip' : (), 'cheby2' : (), 'butter' : (), 'cheby1' : (),  'bessel' : ()}
jointfilterdeg['ellip'] = [x * 180.00/math.pi for x in jointfilter['ellip']]
jointfilterdeg['cheby2'] =  [x * 180.00/math.pi for x in jointfilter['cheby2']]
jointfilterdeg['butter'] = [x * 180.00/math.pi for x in jointfilter['butter']]
jointfilterdeg['cheby1'] = [x * 180.00/math.pi for x in jointfilter['cheby1']]
jointfilterdeg['bessel'] = [x * 180.00/math.pi for x in jointfilter['bessel']]
jointfilterdeg['fir_hamming'] = [x * 180.00/math.pi for x in jointfilter['fir_hamming']]

plot(t,jointdeg, '-o', label="Passive Joint")
pylab.plot(t,jointfilterdeg['ellip'], '-o', label="ellip filter")
pylab.plot(t,jointfilterdeg['cheby2'], '-o', label="cheby2 filter")
pylab.plot(t,jointfilterdeg['butter'], '-o', label="butter filter")
pylab.plot(t,jointfilterdeg['cheby1'], '-o', label="cheby1 filter")
pylab.plot(t,jointfilterdeg['bessel'], '-o', label="bessel filter")
pylab.plot(t,jointfilterdeg['fir_hamming'], '-o', label="fir_hamming filter")
plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'Angle [${}^\circ$]', fontsize=35, fontweight='bold')
plt.legend(prop={'size':25})
grid(True)
plt.show(block=False)


