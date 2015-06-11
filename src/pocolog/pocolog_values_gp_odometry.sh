#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=2

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]} for *.${args[1]}.log files

    #Matrix are organized by rows
    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /gp_odometry.delta_pose_samples_out --field time,pose.translation,pose.cov.data[21],pose.cov.data[22],pose.cov.data[23],pose.cov.data[27],pose.cov.data[28],pose.cov.data[29],pose.cov.data[33],pose.cov.data[34],pose.cov.data[35] > delta_pose_gp_odo_position.${args[1]}.data
    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /gp_odometry.delta_pose_samples_out --field time,pose.orientation,pose.cov.data[0],pose.cov.data[1],pose.cov.data[2],pose.cov.data[6],pose.cov.data[7],pose.cov.data[8],pose.cov.data[12],pose.cov.data[13],pose.cov.data[14] > delta_pose_gp_odo_orientation.${args[1]}.data
    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /gp_odometry.delta_pose_samples_out --field time,velocity.vel,velocity.cov.data[21],velocity.cov.data[22],velocity.cov.data[23],velocity.cov.data[27],velocity.cov.data[28],velocity.cov.data[29],velocity.cov.data[33],velocity.cov.data[34],velocity.cov.data[35] > delta_pose_gp_odo_velocity.${args[1]}.data
    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /gp_odometry.delta_pose_samples_out --field time,velocity.rot,velocity.cov.data[0],velocity.cov.data[1],velocity.cov.data[2],velocity.cov.data[6],velocity.cov.data[7],velocity.cov.data[8],velocity.cov.data[12],velocity.cov.data[13],velocity.cov.data[14] > delta_pose_gp_odo_angular_velocity.${args[1]}.data
    echo 'Delta Pose Odometry Samples... [DONE]'

    # Remove first to lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder and the number of the log file"
    echo "For example pocolog_values_gp_processes . 0 to process *.0.log files"
fi

