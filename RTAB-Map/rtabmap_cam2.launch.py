from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    parameters = [{
        'frame_id': 'cam2_color_optical_frame',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'wait_imu_to_init': False,
        'use_imu': True,
        'queue_size': 30,
        'sync_queue_size': 30,
    }]

    remappings = [
        ('imu',             '/imu/data'),
        ('rgb/image',       '/cam2/color/image_raw'),
        ('rgb/camera_info', '/cam2/color/camera_info'),
        ('depth/image',     '/cam2/aligned_depth_to_color/image_raw'),
    ]

    return LaunchDescription([

        SetParameter(name='use_sim_time', value=True),

        # IMU filter: calcola orientamento da gyro+accel di cam2
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
                'use_sim_time': True,
            }],
            remappings=[('imu/data_raw', '/cam2/imu')],
        ),

        # Odometria visuale RGB-D + IMU
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['--delete_db_on_start'],
        ),

        # SLAM con loop closure e mappa
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['--delete_db_on_start'],
        ),
    ])
