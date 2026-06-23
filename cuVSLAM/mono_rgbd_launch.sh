#!/bin/bash

# Launch file
LAUNCH_FILENAME=mono_rgbd.launch.py

# Copying the python launch file into the isaac_ros_visual_slam launch folder
sudo cp "${LAUNCH_FILENAME}" /opt/ros/"${ROS_DISTRO}"/share/isaac_ros_visual_slam/launch

# Launching the isaac_ros_visual_slam script
ros2 launch isaac_ros_visual_slam "${LAUNCH_FILENAME}"
