from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    num_cameras = 2

    # infra1 = left (image_0), infra2 = right (image_1)
    remapping_list = [
        ('/visual_slam/image_0', '/cam2/infra1/image_rect_raw'),
        ('/visual_slam/camera_info_0', '/cam2/infra1/camera_info'),
        ('/visual_slam/image_1', '/cam2/infra2/image_rect_raw'),
        ('/visual_slam/camera_info_1', '/cam2/infra2/camera_info'),
        ('/visual_slam/imu', '/cam2/imu'),
    ]

    optical_frames = [
        'cam2_infra1_optical_frame',
        'cam2_infra2_optical_frame',
    ]

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_cuvslam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # 0 = solo stereo, 1 = stereo + IMU
            'tracking_mode': 0,
            'enable_image_denoising': False,
            'rectified_images': True,
            'base_frame': 'cam2_link',
            'image_jitter_threshold_ms': 150.,
            'imu_frame': 'cam2_gyro_optical_frame',
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,

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
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return LaunchDescription([visual_slam_launch_container])
