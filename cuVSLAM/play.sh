#!/bin/bash

if [ $# -ne 1 ]; then
	echo "Usage: $0 <bag file to play>"
	exit 1
fi

ros2 bag play $1 --clock --start-paused
