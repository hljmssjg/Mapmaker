#!/bin/bash

# Example script for calling mapper. Don't forget to make it executable (chmod +x mapper)
# Change the last line (python3 Mapper.py ...) to suit your needs
# Author: Ola Ringdahl
#
# Inputs:
# url specifies the address and port to the machine running MRDS. 
# x1, y1, x2, y2 give the coordinates of the lower left and upper right corners of the area the robot should explore and map.
# showGUI: 1 should show the map, 0 should not 

if [ "$#" -ne 6 ]; then
    echo "Usage: ./mapper url x1 y1 x2 y2 showGUI"
    exit 
fi

url="$1"
x1="$2" 
y1="$3" 
x2="$4" 
y2="$5"
showGUI="$6"

# Run the program
python3 Mapper.py $url $x1 $y1 $x2 $y2 $showGUI
