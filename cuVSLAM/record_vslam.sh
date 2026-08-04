#!/bin/bash

if [ $# -ne 1 ]; then
	echo "Usage: $0 <output folder file>"
	exit -1
fi

outpath=$1

ros2 bag record -e "(/tf(.*)|(/visual.slam/(.*))|/cam2/color/image.raw)" -o "$1" 
