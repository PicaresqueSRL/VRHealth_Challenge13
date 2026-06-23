#!/bin/bash

if [ $# -ne 1 ]; then
	echo "Usage: $0 <output folder file>"
	exit 255
fi

ros2 bag record -e "(/tf(.*)|(/visual.slam/(.*)))" -o "$1" 
