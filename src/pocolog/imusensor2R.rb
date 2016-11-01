##########################################################################
# @brief Ruby script to convert from IMU Sensor to R variables.
#
# @section DESCRIPTION
# This script load a particular stream of a rock log and convert to a
# R workspace. In order to use the values inside the R computing software
# Name of the RData file: <log_name>-<stream_name>.RData
#
# @author Javier Hidalgo Carrio | DFKI RIC Bremen | javier.hidalgo_carrio@dfki.de
# @date August 2011.
# @version 1.0.
#
# @section LICENSE
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details at
# http://www.gnu.org/copyleft/gpl.html
##########################################################################


#! /usr/bin/env ruby

require 'pocolog'
require 'rsruby'
include Pocolog

# Chack arguments
if ARGV.size < 2 then 
    puts "usage: imusensor2R.rb <imu_sensor_data_log> <data_stream>"
    exit
end

#R Ruby binding
r = RSRuby.instance

#Open the Log file
file = Logfiles.new File.open(ARGV[0])
data_stream = file.stream(ARGV[1])


# Ruby variables
time = Array.new
acc = Array.new
gyro = Array.new
mag = Array.new

# Reading Samples
data_stream.samples.each do |realtime, logical,sample|

  time.push [sample.time.to_f, sample.time.usec]
  acc.push  [sample.acc.x, sample.acc.y, sample.acc.z]
  gyro.push  [sample.gyro.x, sample.gyro.y, sample.gyro.z]
  mag.push  [sample.mag.x, sample.mag.y, sample.mag.z]
  
end

#Writing into R variables
r.assign('time', time)
r.assign('acc', acc)
r.assign('gyro', gyro)
r.assign('mag', mag)


filename = String.new
filename = ARGV[0]+"-".concat(ARGV[1])+".RData"
puts "creating".concat(filename)

#Saving into a R workspace file
r.save_image(file=filename)
