#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=2

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]} for *.${args[1]}.log files
    pocolog ${args[0]}/contact_odometry.${args[1]}.log -s /contact_odometry.odometry_samples --field time,position,cov_position > pose_contact_odometry_position.${args[1]}.data
    pocolog ${args[0]}/contact_odometry.${args[1]}.log -s /contact_odometry.odometry_samples --field time,orientation,cov_orientation > pose_contact_odometry_orientation.${args[1]}.data
    echo 'Pose Contact Point Odometry Samples... [DONE]'

    pocolog ${args[0]}/contact_odometry.${args[1]}.log -s /contact_odometry.odometry_delta_samples --field time,position,cov_position > delta_pose_contact_odometry_position.${args[1]}.data
    pocolog ${args[0]}/contact_odometry.${args[1]}.log -s /contact_odometry.odometry_delta_samples --field time,orientation,cov_orientation > delta_pose_contract_odometry_orientation.${args[1]}.data
    echo 'Delta Contact Point Odometry Samples... [DONE]'

    # Remove first to lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder and the number of the log file"
    echo "For example pocolog_values_exoter_contact_point_odometry . 0 to process *.0.log files"
fi

