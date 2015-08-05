#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=1

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]}

    pocolog ${args[0]} -s /stim300.orientation_samples_out --field time,orientation,cov_orientation > pose_imu_orientation.0.data
    pocolog ${args[0]} -s /stim300.calibrated_sensors --field time,acc > pose_imu_acceleration.0.data
    pocolog ${args[0]} -s /stim300.calibrated_sensors --field time,gyro > pose_imu_angular_velocity.0.data
    pocolog ${args[0]} -s /stim300.calibrated_sensors --field time,mag > pose_imu_inclinometers.0.data
    pocolog ${args[0]} -s /vicon.pose_samples --field time,orientation,cov_orientation > pose_ref_orientation.0.data
    echo 'Attitude Samples... [DONE]'

    # Remove first to lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder"
fi

