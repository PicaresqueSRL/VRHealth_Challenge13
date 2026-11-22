"""cuVSLAM multi-camera cam2 + cam1 IR stereo, no IMU, localization runs (localize_query.py --config cam1_cam2_stereo).

Same node parameters, cameras and static TF cam2_link -> cam1_link (x -13.4 mm, z -63.4 mm, pitch +49.3 deg) as
cuvslam_stereo_cam1_cam2.launch.py (the launch of the slam_csv cam1_cam2_stereo runs): 4 cameras (cam2 = image_0/1,
cam1 = image_2/3), tracking_mode 0 (multicamera, no IMU), sync_ms 35 (cam1 and cam2 are not hardware synchronized),
base_frame cam2_link. Changes:
  - slam_max_map_size: large, so the saved map keeps the whole capture (cuVSLAM default 300 poses);
  - visualization off (enable_slam_visualization / landmarks / observations), to keep the load low;
  - verbosity 1 (as cuvslam_stereo_imu_cam2_localize.launch.py);
  - localizer search grid: horizontal radius 4 m, vertical radius 1 m, steps 0.5 / 0.25 m, angular step 10 deg
    (same as cuvslam_stereo_imu_cam2_localize.launch.py).

Usage:
  ros2 launch cuvslam_stereo_cam1_cam2_localize.launch.py [sync_ms:=35.0]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    num_cameras = 4

    # stereo pairs: cam2 = image_0/1, cam1 = image_2/3 (infra1 = left, infra2 = right)
    remapping_list = [
        ('/visual_slam/image_0', '/cam2/infra1/image_rect_raw'),
        ('/visual_slam/camera_info_0', '/cam2/infra1/camera_info'),
        ('/visual_slam/image_1', '/cam2/infra2/image_rect_raw'),
        ('/visual_slam/camera_info_1', '/cam2/infra2/camera_info'),
        ('/visual_slam/image_2', '/cam1/infra1/image_rect_raw'),
        ('/visual_slam/camera_info_2', '/cam1/infra1/camera_info'),
        ('/visual_slam/image_3', '/cam1/infra2/image_rect_raw'),
        ('/visual_slam/camera_info_3', '/cam1/infra2/camera_info'),
    ]

    optical_frames = [
        'cam2_infra1_optical_frame',
        'cam2_infra2_optical_frame',
        'cam1_infra1_optical_frame',
        'cam1_infra2_optical_frame',
    ]

    sync_ms = DeclareLaunchArgument(
        'sync_ms', default_value='35.0',
        description='max time difference (ms) between the images of one frame')

    # the bags have no TF between the two cameras: cam2_link -> cam1_link
    cam1_to_cam2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam2_to_cam1_tf',
        arguments=['--x', '-0.0134', '--y', '0.0', '--z', '-0.0634',
                   '--roll', '0.0', '--pitch', '0.8604', '--yaw', '0.0',
                   '--frame-id', 'cam2_link', '--child-frame-id', 'cam1_link'],
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_cuvslam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # 0 = stereo (multi-camera), no IMU
            'tracking_mode': 0,
            'enable_image_denoising': False,
            'rectified_images': True,
            'base_frame': 'cam2_link',
            'image_jitter_threshold_ms': 150.,
            'sync_matching_threshold_ms': ParameterValue(LaunchConfiguration('sync_ms'), value_type=float),

            'enable_slam_visualization': False,
            'enable_landmarks_view': False,
            'enable_observations_view': False,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
            'enable_localization_n_mapping': True,
            'enable_debug_mode': False,
            'verbosity': 1,
            'num_cameras': num_cameras,
            'min_num_images': num_cameras,
            'camera_optical_frames': optical_frames,

            # mapping run: keep the whole capture in the map
            'slam_max_map_size': 100000,

            # localize_in_map search grid (same as cuvslam_stereo_imu_cam2_localize.launch.py)
            'localizer_horizontal_radius': 4.0,
            'localizer_vertical_radius': 1.0,
            'localizer_horizontal_step': 0.5,
            'localizer_vertical_step': 0.25,
            'localizer_angular_step': 0.17453292519943295,
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

    return LaunchDescription([sync_ms, cam1_to_cam2, visual_slam_launch_container])