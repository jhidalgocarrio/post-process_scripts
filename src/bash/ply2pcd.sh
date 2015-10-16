#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=1

if [ $# -ge $NUM_ARGUMENTS ]; then

    program="./ply2pcd"

    if ! [ -z ${args[1]} ]; then
        program=${args[1]}
    fi

    echo Converting ply files to pcd in folder ${args[0]}
    for entry in ${args[0]}/*.ply
    do
          echo "$entry"
          filename="${entry%.*}"
          extension="${entry##*.}"
          ${program} $entry "$filename.pcd"
          sed -i 's/VIEWPOINT 0 0 0 0 0.5 0 0/VIEWPOINT 0 0 0 1 0 0 0/g' "$filename.pcd"
    done

else
    echo Number of arguments passed: $# You need at least ${NUM_ARGUMENTS} arguments.
    echo "Folder containing ply files to convert"
    echo "$ ply2pcd.sh <folder_name>"
fi

