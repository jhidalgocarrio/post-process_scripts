#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=3

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Merging files starting with ${args[0]}/${args[1]}
    mkdir -p ${args[0]}/${args[2]}
    cat  ${args[0]}/${args[1]}.*.data > ${args[0]}/${args[2]}/${args[1]}.0.data

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the folder, the regex to find the file name and the target folder"
    echo "For example to merge all delta_pose_odo_angular_velocity in the current path:"
    echo "$ merge_gpdata_values . delta_pose_odo_angular_velocity ."
fi

