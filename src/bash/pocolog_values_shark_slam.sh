#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=2

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]} for *.${args[1]}.log files

    pocolog ${args[0]}/shark_slam.${args[1]}.log -s /shark_slam.pose_samples_out --field time,position,cov_position > pose_shark_slam_position.${args[1]}.data
    pocolog ${args[0]}/shark_slam.${args[1]}.log -s /shark_slam.pose_samples_out --field time,orientation,cov_orientation > pose_shark_slam_orientation.${args[1]}.data
    echo 'SLAM Pose Samples... [DONE]'


    # Remove first two lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder and the number of the log file"
    echo "For example pocolog_values_shark_slam . 0 to process *.0.log files"
fi

