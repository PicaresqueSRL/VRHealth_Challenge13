# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0
#
# Stereo (infra1 + infra2) launch for a recorded cam1 D455 rosbag.
# cuVSLAM 3.2.x requires at least one stereo pair; the RealSense IR streams
# provide exactly that. RGBD (mono + depth) is NOT accepted by this version.

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    num_cameras = 2

    # infra1 = left (image_0), infra2 = right (image_1)
    remapping_list = [
        ('/visual_slam/image_0', '/cam1/infra1/image_rect_raw'),
        ('/visual_slam/camera_info_0', '/cam1/infra1/camera_info'),
        ('/visual_slam/image_1', '/cam1/infra2/image_rect_raw'),
        ('/visual_slam/camera_info_1', '/cam1/infra2/camera_info'),
    ]

    # Verify these frame names against your bag (see instructions).
    optical_frames = [
        'cam1_infra1_optical_frame',
        'cam1_infra2_optical_frame',
    ]

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'use_sim_time': True,
            'tracking_mode': 0,            # 0 = stereo (no IMU)
            'enable_image_denoising': False,
            'rectified_images': True,      # IR image_rect_raw are already rectified
            'image_jitter_threshold_ms': 100.,
            'base_frame': 'cam1_link',
            'enable_imu_fusion': False,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
            'enable_localization_n_mapping': True,
            'enable_debug_mode': False,
            'num_cameras': num_cameras,
            'min_num_images': num_cameras,
            'camera_optical_frames': optical_frames,
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