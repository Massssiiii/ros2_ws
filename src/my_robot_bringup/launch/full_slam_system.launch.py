from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # === LiDAR SLLIDAR (C1) ===
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py'
            )
        ),
        launch_arguments={'serial_port': '/dev/ttyUSB0'}.items(),
    )

    # === Node du filtre LiDAR ===
    lidar_filter_node = Node(
        package='lidar_chassis_filter',
        executable='lidar_chassis_filter_node',
        name='lidar_chassis_filter',
        output='screen',
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': '/scan_filtered',
            'min_range': 0.25,
            'forbidden_zones': "[[-0.8, -0.6], [0.6, 0.8], [2.3, 2.5], [-2.5, -2.3]]",
        }],
    )

    return LaunchDescription([

        # === Micro-ROS Agent ===
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyUSB1'],
        ),

        # === LiDAR ===
        lidar_launch,

        # === Filtrage du scan (ton node custom) ===
        lidar_filter_node,

        # === TF statiques ===
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom_rf2o', 'base_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        ),

        # === RF2O Laser Odometry (utilise scan filtré) ===
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan_filtered',  # <-- modifié ici !
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom_rf2o',
                'publish_tf': True,
                'init_pose_from_topic': '',
            }],
        ),

        # === SLAM Toolbox (démarre après le LiDAR + filtre) ===
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[{
                        'autostart': True,
                        'use_sim_time': False,
                        'odom_frame': 'odom_rf2o',
                        'base_frame': 'base_link',
                        'map_frame': 'map',
                        'map_update_interval': 1.0,
                        'queue_size': 100,
                    }],
                    remappings=[('scan', '/scan_filtered')],  # <-- modifié ici !
                )
            ]
        ),
    ])
