#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=1

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]}

    pocolog ${args[0]}/dagon_left_base.0.log -s imu_stim300.calibrated_sensors --field time,acc > stim300_imu_acc.0.data
    pocolog ${args[0]}/dagon_left_base.0.log -s imu_stim300.calibrated_sensors --field time,gyro > stim300_imu_gyro.0.data
    pocolog ${args[0]}/dagon_left_base.0.log -s imu_fog.calibrated_sensors --field time,acc > kvh_imu_acc.0.data
    pocolog ${args[0]}/dagon_left_base.0.log -s imu_fog.calibrated_sensors --field time,gyro > kvh_imu_gyro.0.data
    echo 'Attitude Samples... [DONE]'

    # Remove first to lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder"
fi

