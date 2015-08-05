#!/bin/bash
# use predefined variables to access passed arguments
args=("$@")

NUM_ARGUMENTS=1

if [ $# -ge $NUM_ARGUMENTS ]; then

    program="sfdp"

    if ! [ -z ${args[1]} ]; then
        program=${args[1]}
    fi

    echo Converting dot files in folder ${args[0]}
    for entry in ${args[0]}/*.dot
    do
          echo "$entry"
          filename="${entry%.*}"
          extension="${entry##*.}"
          if [ $program == "sfdp" ]; then
            ${program} -x -Goverlap=scale -Tpng $entry -o "$filename.png"
          elif [ $program == "twopi" ]; then
            ${program} -x -Tpng $entry -o "$filename.png"
          fi
    done

else
    echo Number of arguments passed: $# You need at least ${NUM_ARGUMENTS} arguments.
    echo "Folder containing dot files to convert"
    echo "$ dot2png.sh <folder_name>"
fi

