# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
  TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import yaml


def generate_launch_description():
  num_cameras = 1
  remapping_list = [
    ('/visual_slam/image_0', '/cam2/color/image_raw'),
    ('/visual_slam/camera_info_0', '/cam2/color/camera_info'),
    ('/visual_slam/depth_0', '/cam2/aligned_depth_to_color/image_raw'),
    ('/visual_slam/imu', '/cam2/imu'),
  ]
  optical_frames = [
    'cam2_color_optical_frame'
  ]
  visual_slam_node = ComposableNode(
    name='visual_slam_node',
    package='isaac_ros_visual_slam',
    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
    parameters=[{
      'use_sim_time': True,
      'tracking_mode': 2, #1, # VIO (IMU fusion)
      'enable_image_denoising': False,
      'rectified_images': False,
      'image_jitter_threshold_ms': 19.,
      'base_frame': 'cam2_link',
      'imu_frame': 'cam2_gyro_optical_frame',
      'gyro_noise_density': 0.000244,
      'gyro_random_walk': 0.000019393,
      'accel_noise_density': 0.001862,
      'accel_random_walk': 0.003,
      'calibration_frequency': 200.0,
      'enable_imu': True,
      'enable_slam_visualization': True,
      'enable_landmarks_view': True,
      'enable_observations_view': True,
      'enable_ground_constraint_in_odometry': False,
      'enable_ground_constraint_in_slam': False,
      'enable_localization_n_mapping': True,
      'enable_debug_mode': False,
      'num_cameras': num_cameras,
      'min_num_images': num_cameras,
      'cameras_optical_frames': optical_frames,
    }],
    remappings=remapping_list,
  )

  visual_slam_launch_container = ComposableNodeContainer(
    name='visual_slam_launch_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=([visual_slam_node]),
    output='screen',
  )

  return LaunchDescription([visual_slam_launch_container])
