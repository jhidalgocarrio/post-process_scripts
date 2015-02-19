#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=1

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]}
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.joints_samples_out --field time,elements[].position > joints_position.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.joints_samples_out --field time,elements[].speed > joints_speed.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.joints_samples_out --field time,elements[].effort > joints_effort.0.data
    echo 'Joints Samples... [DONE]'

    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.pose_reference_samples_out --field time,position,cov_position > pose_ref_position.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.pose_reference_samples_out --field time,velocity,cov_velocity > pose_ref_velocity.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.pose_reference_samples_out --field time,orientation,cov_orientation > pose_ref_orientation.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.pose_reference_samples_out --field time,angular_velocity,cov_angular_velocity > pose_ref_angular_velocity.0.data
    echo 'Pose Reference Samples... [DONE]'

    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.delta_pose_reference_samples_out --field time,position,cov_position > delta_pose_ref_position.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.delta_pose_reference_samples_out --field time,velocity,cov_velocity > delta_pose_ref_velocity.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.delta_pose_reference_samples_out --field time,orientation,cov_orientation > delta_pose_ref_orientation.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.delta_pose_reference_samples_out --field time,angular_velocity,cov_angular_velocity > delta_pose_ref_angular_velocity.0.data
    echo 'Delta Pose Reference Samples... [DONE]'

    pocolog ${args[0]}/exoter_odometry.0.log -s /exoter_odometry.pose_samples_out --field time,position,cov_position > pose_odo_position.0.data
    pocolog ${args[0]}/exoter_odometry.0.log -s /exoter_odometry.pose_samples_out --field time,velocity,cov_velocity > pose_odo_velocity.0.data
    pocolog ${args[0]}/exoter_odometry.0.log -s /exoter_odometry.pose_samples_out --field time,orientation,cov_orientation > pose_odo_orientation.0.data
    pocolog ${args[0]}/exoter_odometry.0.log -s /exoter_odometry.pose_samples_out --field time,angular_velocity,cov_angular_velocity > pose_odo_angular_velocity.0.data
    echo 'Pose Odometry Samples... [DONE]'

    pocolog ${args[0]}/exoter_odometry.0.log -s /exoter_odometry.delta_pose_samples_out --field time,position,cov_position > delta_pose_odo_position.0.data
    pocolog ${args[0]}/exoter_odometry.0.log -s /exoter_odometry.delta_pose_samples_out --field time,velocity,cov_velocity > delta_pose_odo_velocity.0.data
    pocolog ${args[0]}/exoter_odometry.0.log -s /exoter_odometry.delta_pose_samples_out --field time,orientation,cov_orientation > delta_pose_odo_orientation.0.data
    pocolog ${args[0]}/exoter_odometry.0.log -s /exoter_odometry.delta_pose_samples_out --field time,angular_velocity,cov_angular_velocity > delta_pose_odo_angular_velocity.0.data
    echo 'Delta Pose Odometry Samples... [DONE]'

    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.orientation_samples_out --field time,orientation,cov_orientation > pose_imu_orientation.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.inertial_samples_out --field time,gyro > pose_imu_angular_velocity.0.data
    pocolog ${args[0]}/exoter_perception.0.log -s /localization_frontend.inertial_samples_out --field time,acc > pose_imu_acceleration.0.data
    echo 'Attitude Samples... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder"
fi

