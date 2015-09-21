#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=2

if [ $# -ge $NUM_ARGUMENTS ]; then

    pocolog ${args[0]}/exoter_localization.${args[1]}.log -s /msc_localization.filter_info_out --field  number_features_added,add_features_execution_time > envire_add_new_nodes.${args[1]}.data
    pocolog ${args[0]}/exoter_localization.${args[1]}.log -s /msc_localization.filter_info_out --field  number_features_removed,remove_features_execution_time > envire_remove_existing_nodes.${args[1]}.data
    echo 'Delta Pose Skid Odometry Samples... [DONE]'

    # Remove first to lines of the files (pocolog headers)
    sed -i '1,2d' *.data
    echo 'Remove file headers... [DONE]'

else
    echo Number of arguments passed: $#
    echo "You need to specify the path to the log-folder and the number of the log file"
    echo "For example pocolog_values_gaussian_processes . 0 to process *.0.log files"
fi

