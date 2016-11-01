##############################################
# Script to convert CSV data 
# into rock log format.
##############################################


require 'orocos'
require 'pocolog'
require 'vizkit'
require 'eigen'

Orocos.load
Orocos.load_typekit 'base'


if ARGV.size < 3 then 
    puts "usage: acc_input_file gyros_input_file magn_input_file time_input_file euler_input_file output_file"
    exit
end

sensors_output = Pocolog::Logfiles.create(ARGV[5], Orocos.registry )
truth = Orocos.registry.get("/base/samples/RigidBodyState_m")
imu = Orocos.registry.get("/base/samples/IMUSensors_m")
euler_stream1 = sensors_output.stream("truth.attitude",truth, true)
imu_stream2 = sensors_output.stream("imu.imu",imu, true)

#Open the file with the logs
facc = File.new(ARGV[0])
fgyros = File.new(ARGV[1])
fmagn = File.new(ARGV[2])
ftime = File.new(ARGV[3])
feuler = File.new(ARGV[4])

starttime = Time.new
prevtime = 0.00
time_flag = false

#Data types in the log
imu = Types::Base::Samples::IMUSensors.new
truth = Types::Base::Samples::RigidBodyState.new


# line_acc = facc.readline
# line_gyros = fgyros.readline
# line_magn = fmagn.readline
# line_time = ftime.readline
# line_euler = feuler.readline

begin
while (line_acc = facc.readline) # Same length in all the files

    tokens_acc = line_acc.split("\t")
    
    # Read all files lines 
    line_gyros = fgyros.readline
    line_magn = fmagn.readline
    line_time = ftime.readline
    line_euler = feuler.readline
    
    line_acc.chomp
    line_gyros.chomp
    line_magn.chomp
    line_time.chomp
    line_euler.chomp
    
    
    # Split in tokens
    tokens_gyros = line_gyros.split("\t")
    tokens_magn = line_magn.split("\t")
    tokens_time = line_time.split("\t")
    tokens_euler = line_euler.split("\t")
    
    if (time_flag == false)
	prevtime = tokens_time[0].to_f
	time_flag = true
    end
    
#     puts "acc\n"
#     $stdout.print tokens_acc[0].to_f, "\n"
#     puts "gyros\n"
#     $stdout.print tokens_gyros[0].to_f, "\n"
#     puts "magn\n"
#     $stdout.print tokens_magn[0].to_f, "\n"
    puts "First, time "
    $stdout.print tokens_time[0].to_f, " seconds \n"
    puts "imu.time.to_f", imu.time.to_f, "\n"
    puts "euler: "
    $stdout.print tokens_euler[0].to_f, " ", tokens_euler[1].to_f, " ", tokens_euler[2].to_f, "\n"
    diff_time = (tokens_time[0].to_f - prevtime)
    puts "diff time: "
    $stdout.print diff_time, "\n"
    #Fill the IMUSensors data type
    starttime = starttime + diff_time
    imu.time = starttime
        
    $stdout.print "Time(imu): ", imu.time.to_f, "\n"
    $stdout.print "Time: ", starttime, "\n"
	
    imu.acc[0] = tokens_acc[0].to_f
    imu.acc[1] = tokens_acc[1].to_f
    imu.acc[2] = tokens_acc[2].to_f
    
    imu.gyro[0] = tokens_gyros[0].to_f
    imu.gyro[1] = tokens_gyros[1].to_f
    imu.gyro[2] = tokens_gyros[2].to_f
    
    imu.mag[0] = tokens_magn[0].to_f
    imu.mag[1] = tokens_magn[1].to_f
    imu.mag[2] = tokens_magn[2].to_f
    
    #Fill the Rigidbodystate (euler(0) is pitch euler(1) is roll euler(2) is yaw) ????
    truth.time = starttime
        
    v= Eigen::Vector3.new(tokens_euler[2].to_f, tokens_euler[1].to_f, tokens_euler[0].to_f) 
    puts v
    v = v * Math::PI/180.00
    puts v
    truth.orientation = Eigen::Quaternion.from_euler(v, 2,1,0)
    
    prevtime = tokens_time[0].to_f

    
    euler_stream1.write(truth.time,truth.time,truth)
    imu_stream2.write(imu.time,imu.time,imu)
end

rescue EOFError
    facc.close
    fgyros.close
    fmagn.close
    ftime.close
    feuler.close
end
