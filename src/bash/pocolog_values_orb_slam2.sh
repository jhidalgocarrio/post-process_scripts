#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=2

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]} for *.${args[1]}.log files

    pocolog ${args[0]}/orb_slam2.${args[1]}.log -s /orb_slam2.task_info_out > task_info_adaptive_slam.${args[1]}.data
    echo 'SLAM Task information... [DONE]'

    pocolog ${args[0]}/orb_slam2.${args[1]}.log -s /orb_slam2.keyframe_pose_samples_out --field time,position,cov_position > pose_keyframe_orb_slam2_position.${args[1]}.data
    pocolog ${args[0]}/orb_slam2.${args[1]}.log -s /orb_slam2.keyframe_pose_samples_out --field time,orientation,cov_orientation > pose_keyframe_orb_slam2_orientation.${args[1]}.data
    pocolog ${args[0]}/orb_slam2.${args[1]}.log -s /orb_slam2.pose_samples_out --field time,position,cov_position > pose_orb_slam2_position.${args[1]}.data
    pocolog ${args[0]}/orb_slam2.${args[1]}.log -s /orb_slam2.pose_samples_out --field time,orientation,cov_orientation > pose_orb_slam2_orientation.${args[1]}.data
    echo 'SLAM Pose Samples... [DONE]'

    pocolog ${args[0]}/orb_slam2.${args[1]}.log -s /orb_slam2.task_info_out  > task_orb_slam2_info.data.${args[1]}.data
    echo 'Task information... [DONE]'

    pocolog ${args[0]}/localization_frontend.${args[1]}.log -s /localization_frontend.pose_reference_samples_out --field time,position,cov_position > pose_ref_position.${args[1]}.data
    pocolog ${args[0]}/localization_frontend.${args[1]}.log -s /localization_frontend.pose_reference_samples_out --field time,orientation,cov_orientation > pose_ref_orientation.${args[1]}.data
    echo 'Pose Reference Samples... [DONE]'

    pocolog ${args[0]}/localization_frontend.${args[1]}.log -s /localization_frontend.delta_pose_reference_samples_out --field time,position,cov_position > delta_pose_ref_position.${args[1]}.data
    pocolog ${args[0]}/localization_frontend.${args[1]}.log -s /localization_frontend.delta_pose_reference_samples_out --field time,orientation,cov_orientation > delta_pose_ref_orientation.${args[1]}.data
    echo 'Delta Pose Reference Samples... [DONE]'

    pocolog ${args[0]}/localization_frontend.${args[1]}.log -s /localization_frontend.reaction_forces_samples_out > scaled_odo_reaction_forces.${args[1]}.data
    echo 'Estimated Reaction Forces... [DONE]'

    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /exoter_odometry.pose_samples_out --field time,position,cov_position > pose_odo_position.${args[1]}.data
    pocolog ${args[0]}/exoter_odometry.${args[1]}.log -s /exoter_odometry.pose_samples_out --field time,orientation,cov_orientation > pose_odo_orientation.${args[1]}.data
    echo 'Odometry Pose Samples... [DONE]'

    pocolog ${args[0]}/gpy_gp_odometry.${args[1]}.log -s /gpy_gp_odometry.delta_pose_samples_out --field time,position,cov_position > delta_pose_odo_position.${args[1]}.data
    pocolog ${args[0]}/gpy_gp_odometry.${args[1]}.log -s /gpy_gp_odometry.delta_pose_samples_out --field time,orientation,cov_orientation > delta_pose_odo_orientation.${args[1]}.data
    echo 'GP-Odometry Delta Pose Samples... [DONE]'

    pocolog ${args[0]}/localization_frontend.${args[1]}.log -s /localization_frontend.world_to_navigation_out --field time,position,cov_position > pose_world_to_navigation_position.${args[1]}.data
    pocolog ${args[0]}/localization_frontend.${args[1]}.log -s /localization_frontend.world_to_navigation_out --field time,orientation,cov_orientation > pose_world_to_navigation_orientation.${args[1]}.data
    echo 'World to Navigation Samples... [DONE]'

    # Remove first two lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder and the number of the log file"
    echo "For example pocolog_values_exoter_localization . 0 to process *.0.log files"
fi

