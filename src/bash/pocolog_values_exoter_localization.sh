#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=2

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]} for *.${args[1]}.log files
    pocolog ${args[0]}/exoter_perception.${args[1]}.log -s /localization_frontend.pose_reference_samples_out --field time,position,cov_position > pose_ref_position.${args[1]}.data
    pocolog ${args[0]}/exoter_perception.${args[1]}.log -s /localization_frontend.pose_reference_samples_out --field time,orientation,cov_orientation > pose_ref_orientation.${args[1]}.data
    echo 'Pose Reference Samples... [DONE]'

    pocolog ${args[0]}/exoter_perception.${args[1]}.log -s /localization_frontend.delta_pose_reference_samples_out --field time,position,cov_position > delta_pose_ref_position.${args[1]}.data
    pocolog ${args[0]}/exoter_perception.${args[1]}.log -s /localization_frontend.delta_pose_reference_samples_out --field time,orientation,cov_orientation > delta_pose_ref_orientation.${args[1]}.data
    echo 'Delta Pose Reference Samples... [DONE]'

    pocolog ${args[0]}/exoter_perception.${args[1]}.log -s /localization_frontend.reaction_forces_samples_out > scaled_odo_reaction_forces.${args[1]}.data
    echo 'Estimated Reaction Forces... [DONE]'

    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /exoter_odometry.pose_samples_out --field time,position,cov_position > pose_odo_position.${args[1]}.data
    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /exoter_odometry.pose_samples_out --field time,orientation,cov_orientation > pose_odo_orientation.${args[1]}.data
    echo 'Pose Odometry Samples... [DONE]'

    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /exoter_odometry.delta_pose_samples_out --field time,pose.translation,pose.cov.data[21],pose.cov.data[22],pose.cov.data[23],pose.cov.data[27],pose.cov.data[28],pose.cov.data[29],pose.cov.data[33],pose.cov.data[34],pose.cov.data[35] > delta_pose_odo_position.${args[1]}.data
    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /exoter_odometry.delta_pose_samples_out --field time,pose.orientation,pose.cov.data[0],pose.cov.data[1],pose.cov.data[2],pose.cov.data[6],pose.cov.data[7],pose.cov.data[8],pose.cov.data[12],pose.cov.data[13],pose.cov.data[14] > delta_pose_odo_orientation.${args[1]}.data
    echo 'Delta Pose Odometry Samples... [DONE]'

    pocolog ${args[0]}/skid_odometry.${args[1]}.log -s /skid_odometry.odometry_samples --field time,position,cov_position > pose_skid_position.${args[1]}.data
    pocolog ${args[0]}/skid_odometry.${args[1]}.log -s /skid_odometry.odometry_samples --field time,orientation,cov_orientation > pose_skid_orientation.${args[1]}.data
    echo 'Pose Skid Odometry Samples... [DONE]'

    pocolog ${args[0]}/skid_odometry.${args[1]}.log -s /skid_odometry.odometry_delta_samples --field time,position,cov_position > delta_pose_skid_position.${args[1]}.data
    pocolog ${args[0]}/skid_odometry.${args[1]}.log -s /skid_odometry.odometry_delta_samples --field time,orientation,cov_orientation > delta_pose_skid_orientation.${args[1]}.data
    echo 'Delta Pose Skid Odometry Samples... [DONE]'

    # Remove first to lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder and the number of the log file"
    echo "For example pocolog_values_gaussian_processes . 0 to process *.0.log files"
fi

