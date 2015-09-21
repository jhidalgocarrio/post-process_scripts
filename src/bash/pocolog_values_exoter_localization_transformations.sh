#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=2

if [ $# -ge $NUM_ARGUMENTS ]; then
    echo Running the pocolog commands for the directory: ${args[0]} for *.${args[1]}.log files
    pocolog ${args[0]}/exoter_localization.${args[1]}.log -s /localization_frontend.world_to_navigation_out --field time,position,cov_position > pose_world_to_navigation_position.${args[1]}.data
    pocolog ${args[0]}/exoter_localization.${args[1]}.log -s /localization_frontend.world_to_navigation_out --field time,orientation,cov_orientation > pose_world_to_navigation_orientation.${args[1]}.data
    echo 'World to Navigation Samples... [DONE]'

    # Remove first to lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder and the number of the log file"
    echo "For example pocolog_values_exoter_localization_transformation . 0 to process *.0.log files"
fi

