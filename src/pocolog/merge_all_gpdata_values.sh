#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=1

if [ $# -ge $NUM_ARGUMENTS ]; then

    echo Merging files...
    ${args[0]}/merge_gpdata_values.sh . delta_pose_odo_angular_velocity merge
    ${args[0]}/merge_gpdata_values.sh . delta_pose_odo_orientation merge
    ${args[0]}/merge_gpdata_values.sh . delta_pose_odo_position merge
    ${args[0]}/merge_gpdata_values.sh . delta_pose_odo_velocity merge
    ${args[0]}/merge_gpdata_values.sh . delta_pose_odo_velocity merge
    ${args[0]}/merge_gpdata_values.sh . delta_pose_ref_angular_velocity merge
    ${args[0]}/merge_gpdata_values.sh . delta_pose_ref_orientation merge
    ${args[0]}/merge_gpdata_values.sh . delta_pose_ref_position merge
    ${args[0]}/merge_gpdata_values.sh . delta_pose_ref_velocity merge
    ${args[0]}/merge_gpdata_values.sh . joints_position merge
    ${args[0]}/merge_gpdata_values.sh . joints_speed merge
    ${args[0]}/merge_gpdata_values.sh . joints_effort merge
    ${args[0]}/merge_gpdata_values.sh . pose_imu_orientation merge
    ${args[0]}/merge_gpdata_values.sh . pose_imu_angular_velocity merge
    ${args[0]}/merge_gpdata_values.sh . pose_imu_acceleration merge
    ${args[0]}/merge_gpdata_values.sh . pose_odo_position merge
    ${args[0]}/merge_gpdata_values.sh . pose_odo_velocity merge
    ${args[0]}/merge_gpdata_values.sh . pose_odo_orientation merge
    ${args[0]}/merge_gpdata_values.sh . pose_odo_angular_velocity merge
    ${args[0]}/merge_gpdata_values.sh . pose_ref_position merge
    ${args[0]}/merge_gpdata_values.sh . pose_ref_velocity merge
    ${args[0]}/merge_gpdata_values.sh . pose_ref_orientation merge
    ${args[0]}/merge_gpdata_values.sh . pose_ref_angular_velocity merge

fi


