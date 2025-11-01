from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('my_robot_bringup')
    nav2_config = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    # === LiDAR SLLIDAR ===
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

    # === LiDAR filter node ===
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

    # === Micro-ROS Agent ===
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyUSB1'],
    )

    # === Static TFs (start with small delay to avoid TF=0.0 bug) ===
    tf_nodes = [
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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_basefootprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        ),
    ]
    delayed_tf = TimerAction(period=2.0, actions=tf_nodes)

    # === RF2O Laser Odometry (?? inchangé comme ta version qui marche) ===
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_node',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan_filtered',
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom_rf2o',
            'publish_tf': True,
            'init_pose_from_topic': '',
        }],
        remappings=[('scan', '/scan_filtered')],
    )

    # === Nav2 Bringup (lancé après RF2O et TFs stables) ===
    nav2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'bringup_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'False',
                    'params_file': nav2_config,
                    'map': '/home/rasp/map.yaml'
                }.items(),
            )
        ]
    )

    # === Lifecycle Manager Navigation ===
    lifecycle_manager_nav = TimerAction(
        period=10.0,  # démarre après bringup
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'bond_timeout': 0.0,
                    # ?? On NE met PAS map_server ni amcl ici, pour éviter le conflit avec lifecycle_manager_localization
                    'node_names': [
                        'planner_server',
                        'controller_server',
                        'bt_navigator',
                        'behavior_server',
                        'smoother_server',
                        'waypoint_follower',
                        'velocity_smoother',
                        'global_costmap/global_costmap',
                        'local_costmap/local_costmap'
                    ]
                }]
            )
        ]
    )

    return LaunchDescription([
        micro_ros_agent,
        lidar_launch,
        lidar_filter_node,
        delayed_tf,
        rf2o_node,
        nav2_launch,
        lifecycle_manager_nav
    ])
