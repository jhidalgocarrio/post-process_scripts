#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=1

if [ $# -ge $NUM_ARGUMENTS ]; then

    echo Merging files...
    ./merge_gpdata_values.sh ${args[0]} delta_pose_odo_angular_velocity merged
    ./merge_gpdata_values.sh ${args[0]} delta_pose_odo_orientation merged
    ./merge_gpdata_values.sh ${args[0]} delta_pose_odo_position merged
    ./merge_gpdata_values.sh ${args[0]} delta_pose_odo_velocity merged
    ./merge_gpdata_values.sh ${args[0]} delta_pose_odo_velocity merged
    ./merge_gpdata_values.sh ${args[0]} delta_pose_ref_angular_velocity merged
    ./merge_gpdata_values.sh ${args[0]} delta_pose_ref_orientation merged
    ./merge_gpdata_values.sh ${args[0]} delta_pose_ref_position merged
    ./merge_gpdata_values.sh ${args[0]} delta_pose_ref_velocity merged
    ./merge_gpdata_values.sh ${args[0]} joints_position merged
    ./merge_gpdata_values.sh ${args[0]} joints_speed merged
    ./merge_gpdata_values.sh ${args[0]} joints_effort merged
    ./merge_gpdata_values.sh ${args[0]} pose_imu_orientation merged
    ./merge_gpdata_values.sh ${args[0]} pose_imu_angular_velocity merged
    ./merge_gpdata_values.sh ${args[0]} pose_imu_acceleration merged
    ./merge_gpdata_values.sh ${args[0]} pose_odo_position merged
    ./merge_gpdata_values.sh ${args[0]} pose_odo_velocity merged
    ./merge_gpdata_values.sh ${args[0]} pose_odo_orientation merged
    ./merge_gpdata_values.sh ${args[0]} pose_odo_angular_velocity merged
    ./merge_gpdata_values.sh ${args[0]} pose_ref_position merged
    ./merge_gpdata_values.sh ${args[0]} pose_ref_velocity merged
    ./merge_gpdata_values.sh ${args[0]} pose_ref_orientation merged
    ./merge_gpdata_values.sh ${args[0]} pose_ref_angular_velocity merged

fi


