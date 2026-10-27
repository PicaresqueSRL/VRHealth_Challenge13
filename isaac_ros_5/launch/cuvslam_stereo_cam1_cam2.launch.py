"""cuVSLAM multi-camera: cam2 (forward) + cam1 (down), stereo IR, no IMU, visualization.

cam1 and cam2 are not hardware synchronized: their frames are 5-33 ms apart
depending on the bag, so the sync threshold is raised (argument sync_ms).

Pose of cam1 relative to cam2 (TF cam2_link -> cam1_link):
  translation from the holder CAD (D455_HOLDER.stp): x -13.4 mm, z -63.4 mm
  rotation estimated from the two gyros on all 17 bags: pitch +49.3 deg

Usage:
  ros2 launch cuvslam_stereo_cam1_cam2_bench.launch.py [sync_ms:=35.0]
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

    return LaunchDescription([sync_ms, cam1_to_cam2, visual_slam_launch_container])