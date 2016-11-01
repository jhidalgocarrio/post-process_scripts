##########################################################################
# @brief Ruby script to convert from Rigit body State to R variables.
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
require 'quaternion'
include Pocolog

# Chack arguments
if ARGV.size < 2 then 
    puts "usage: rigidbody2R.rb <rigid_body_data_log> <data_stream>"
    exit
end

#R Ruby binding
r = RSRuby.instance

#Open the Log file
file = Logfiles.new File.open(ARGV[0])
data_stream = file.stream(ARGV[1])


# Ruby variables
time = Array.new
position = Array.new
cov_position = Array.new
orientation = Array.new
cov_orientation = Array.new
velocity = Array.new
cov_velocity = Array.new
angular_velocity = Array.new
cov_angular_velocity = Array.new

# Reading Samples
data_stream.samples.each do |realtime, logical,sample|
  
#   time.push  [sample.time.hour,sample.time.min,sample.time.sec,sample.time.usec]
  time.push [sample.time.to_f, sample.time.usec]
  
  position.push  [sample.position.x, sample.position.y, sample.position.z]
  cov_position.push  [sample.cov_position.data[0], sample.cov_position.data[1], sample.cov_position.data[2],
                           sample.cov_position.data[3], sample.cov_position.data[4], sample.cov_position.data[5],
                           sample.cov_position.data[6], sample.cov_position.data[7], sample.cov_position.data[8]]
  
  if (sample.orientation.w != NaN || sample.orientation.x != NaN || sample.orientation.y != NaN || sample.orientation.z != NaN)
    if (sample.orientation.w != 0.00 && sample.orientation.x != 0.00 && sample.orientation.y != 0.00 && sample.orientation.z != 0.00)
      q = Quaternion.new(sample.orientation.w, sample.orientation.x, sample.orientation.y, sample.orientation.z)
      orientation.push  [q.to_roll, q.to_pitch, q.to_yaw]
      cov_orientation.push  [sample.cov_orientation.data[0], sample.cov_orientation.data[1], sample.cov_orientation.data[2],
			      sample.cov_orientation.data[3], sample.cov_orientation.data[4], sample.cov_orientation.data[5],
			      sample.cov_orientation.data[6], sample.cov_orientation.data[7], sample.cov_orientation.data[8]]
    end
  end
  
  
  velocity.push  [sample.velocity.x, sample.velocity.y, sample.velocity.z]
  cov_velocity.push  [sample.cov_velocity.data[0], sample.cov_velocity.data[1], sample.cov_velocity.data[2],
                           sample.cov_velocity.data[3], sample.cov_velocity.data[4], sample.cov_velocity.data[5],
                           sample.cov_velocity.data[6], sample.cov_velocity.data[7], sample.cov_velocity.data[8]]
  
  
  angular_velocity.push  [sample.angular_velocity.x, sample.angular_velocity.y, sample.angular_velocity.z]
  cov_angular_velocity.push  [sample.cov_angular_velocity.data[0], sample.cov_angular_velocity.data[1], sample.cov_angular_velocity.data[2],
                           sample.cov_angular_velocity.data[3], sample.cov_angular_velocity.data[4], sample.cov_angular_velocity.data[5],
                           sample.cov_angular_velocity.data[6], sample.cov_angular_velocity.data[7], sample.cov_angular_velocity.data[8]]
  
  
  #puts time
  #puts cov_position
  
end


#r.assign('rb', NULL)
r.assign('time', time)
#time = r.as_data_frame(r.time)
#time = r.as_data_frame(r.sapply(r.time, r.seq()))
#r.assign(r.time, time)
#r.time r.__ r.as_data_frame(r.sapply(r.time, r.seq()))
#r.assign(r.names(r.time), r.seq(1, r.length(time)))

#r.names('time') = seq(0, length(time))
r.assign('position', position)
r.assign('cov_position', cov_position)
r.assign('orientation', orientation)
r.assign('cov_orientation', cov_orientation)
r.assign('velocity', velocity)
r.assign('cov_velocity', cov_velocity)
r.assign('angular_velocity', angular_velocity)
r.assign('cov_angular_velocity', cov_angular_velocity)

r.assign('sourceFrame', data_stream.samples.to_a[1][2].sourceFrame)
r.assign('targetFrame', data_stream.samples.to_a[1][2].targetFrame)


filename = String.new
filename = ARGV[0]+"-".concat(ARGV[1])+".RData"
puts filename

r.save_image(file=filename)