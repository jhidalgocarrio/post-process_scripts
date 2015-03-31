#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=1

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]}

    pocolog ${args[0]}/exoter_proprioceptive.0.log -s /imu_stim300.inertial_sensors_out --field time,acc > stim300_imu_acc.0.data
    pocolog ${args[0]}/exoter_proprioceptive.0.log -s /imu_stim300.inertial_sensors_out --field time,gyro > stim300_imu_gyro.0.data
    pocolog ${args[0]}/exoter_proprioceptive.0.log -s /imu_stim300.compensated_sensors_out --field time,acc > pose_imu_acc.0.data
    pocolog ${args[0]}/exoter_proprioceptive.0.log -s /imu_stim300.compensated_sensors_out --field time,gyro > pose_imu_gyro.0.data
    pocolog ${args[0]}/exoter_proprioceptive.0.log -s /imu_stim300.orientation_samples_out --field time,orientation,cov_orientation > pose_imu_orientation.0.data
    pocolog ${args[0]}/exoter_groundtruth.0.log -s /ar_tracking.pose_samples --field time,orientation,cov_orientation > pose_ref_orientation.0.data
    echo 'Attitude Samples... [DONE]'

    # Remove first to lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder"
fi

