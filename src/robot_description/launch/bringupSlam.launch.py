import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Déclaration des arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Récupération des packages nécessaires
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    rplidar_ros_pkg_dir = get_package_share_directory('sllidar_ros2')
    laser_filters_pkg_dir = get_package_share_directory('laser_filters')
    odometry_pkg_dir = get_package_share_directory('rf2o_laser_odometry')
    robot_nav_pkg_dir = get_package_share_directory('robot_nav')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Utilisation du temps de simulation'
        ),

        # Lancer micro_ros_agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB1'],
            output='screen'
        ),

        # Lancer SL-LIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_ros_pkg_dir, 'launch', 'sllidar_c1_launch.py')
            )
        ),

        # Lancer le filtre laser
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(laser_filters_pkg_dir, 'examples', 'range_filter_example.launch.py')
            ),
            launch_arguments={'params_file': '/home/rasp/ros2_ws/src/laser_filters/examples/range_filter_example.yaml'}.items()
        ),

        # Lancer la description du robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_description_pkg_dir, 'launch', 'display.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time, 'ignore_timestamp': 'false'}.items()
        ),

        # Lancer RF2O Laser Odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(odometry_pkg_dir, 'launch', 'rf2o_laser_odometry.launch.py')
            )
        ),

        # Lancer SLAM Toolbox (online_async)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_nav'), 'launch', 'slam_launch.py')
            ),
            launch_arguments={
                #'params_file': '/home/rasp/ros2_ws/src/robot_nav/config/mapper_params_online_async.yaml',
                'use_sim_time': use_sim_time,
            }.items()
      
        )
    ])
