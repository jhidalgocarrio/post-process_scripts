##############################################
# Script to export data file of LRM from ESA #
# into rock log format.
##############################################


require 'orocos'
require 'pocolog'
require 'vizkit'

Orocos.load
Orocos.load_typekit 'base'


if ARGV.size < 3 then 
    puts "usage: input_file input_covfile output_file"
    exit
end

lrmtest_output = Pocolog::Logfiles.create(ARGV[2], Orocos.registry )
lrm_rbs = Orocos.registry.get("/base/samples/RigidBodyState_m")
imt30 = Orocos.registry.get("/base/samples/IMUSensors_m")
lrmtest_stream1 = lrmtest_output.stream("lrm_rbs.attitude",lrm_rbs, true)
lrmtest_stream2 = lrmtest_output.stream("imt30.imu",imt30, true)

#Open the file with the logs
f = File.new(ARGV[0])
covf = File.new(ARGV[1])

starttime = Time.new

#Data types in the log
imt30 = Types::Base::Samples::IMUSensors.new
lrm_rbs = Types::Base::Samples::RigidBodyState.new


line = f.readline
line = f.readline
covline = covf.readline
covline = covf.readline

begin
while (line = f.readline)
    line.chomp
    tokens = line.split("\t")
    
    #$stdout.print tokens[0].to_f, "\n"
    
    #Fill the IMT30 data type
    imt30.time = starttime + tokens[0].to_f
    #$stdout.print imt30.time.asctime, "\n"
	
    imt30.acc[0] = tokens[11].to_f
    imt30.acc[1] = tokens[12].to_f
    imt30.acc[2] = tokens[13].to_f
    
    imt30.gyro[0] = tokens[14].to_f
    imt30.gyro[1] = tokens[15].to_f
    imt30.gyro[2] = tokens[16].to_f
    
    imt30.mag[0] = 0.0
    imt30.mag[1] = 0.0
    imt30.mag[2] = 0.0
    
    #Fill the Rigidbodystate
    lrm_rbs.time = starttime + tokens[0].to_f
    lrm_rbs.orientation.w = tokens[1].to_f
    lrm_rbs.orientation.x = tokens[2].to_f
    lrm_rbs.orientation.y = tokens[3].to_f
    lrm_rbs.orientation.z = tokens[4].to_f
    
    #Position is the orientation in degrees
    lrm_rbs.position[0] = tokens[5].to_f
    lrm_rbs.position[1] = tokens[6].to_f
    lrm_rbs.position[2] = tokens[7].to_f
	
    #RBS velocities are the vicon orientation in degrees
    lrm_rbs.velocity[0] = tokens[17].to_f
    lrm_rbs.velocity[1] = tokens[18].to_f
    lrm_rbs.velocity[2] = tokens[19].to_f
	
    #RBS angular_velocities are the error between AHRS and vicon in degrees
    lrm_rbs.angular_velocity[0] = tokens[20].to_f
    lrm_rbs.angular_velocity[1] = tokens[21].to_f
    lrm_rbs.angular_velocity[2] = tokens[22].to_f
    
    #Covariance matrix of the orientation in quaternion representation (pitch and roll)
    for i in (0..2)
      covline = covf.readline
      covline.chomp
      tokens = covline.split("\t")
      
      #One row
      lrm_rbs.cov_orientation.data[(i*3)+0] = tokens[0].to_f
      lrm_rbs.cov_orientation.data[(i*3)+1] = tokens[1].to_f
      lrm_rbs.cov_orientation.data[(i*3)+2] = tokens[2].to_f

    end
    
    covline = covf.readline
    covline = covf.readline
    covline = covf.readline

    lrmtest_stream1.write(lrm_rbs.time,lrm_rbs.time,lrm_rbs)
    lrmtest_stream2.write(imt30.time,imt30.time,imt30)
end
rescue EOFError
    f.close
    covf.close
end
