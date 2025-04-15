# -*- coding: utf-8 -*-

import subprocess

def run_command(command):
    """Execute une commande en tant que sous-processus."""
    try:
        subprocess.run(command, check=True, shell=True)
    except subprocess.CalledProcessError as e:
        print(f"Erreur lors de l'execution de la commande : {command}")
        print(e)

if __name__ == '__main__':
    # Lancer micro_ros_agent
    run_command('screen -dmS myssesion1 ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 &')

    # Lancer rplidar
    #run_command('screen -dmS myssesion2 ros2 launch rplidar_ros rplidar_c1_launch.py &')
    
    #lancer slildar
    run_command('screen -dmS myssesion2 ros2 launch sllidar_ros2 sllidar_c1_launch.py &')
    
    
    # Lancer filtre
    run_command('screen -dmS mysession4 ros2 launch laser_filters range_filter_example.launch.py params_file:="/home/rasp/ros2_ws/src/laser_filters/examples/range_filter_example.yaml"')

    # Lancer la description du robot
    run_command('screen -dmS myssesion5 ros2 launch robot_description display.launch.py use_sim_time:=false ignore_timestamp:=false &')

    # Lancer teleop_twist_keyboard
    #run_command('screen -dmS myssesion6 ros2 run teleop_twist_keyboard teleop_twist_keyboard &')

    # Lancer rf2o_laser_odometry
    run_command('screen -dmS myssesion7 ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &')

    # Lancer slam_toolbox pour la cartographie
    #run_command('screen -dmS myssesion6 ros2 launch slam_toolbox online_async_launch.py params_file:=./src/nav/mapper_params_online_async.yaml use_sim_time:=False &')
    
    # test pour essayer de resoudre le probleme des time stamp
    #run_command('screen -dmS myssesion8 ros2 launch slam_toolbox online_async_launch.py params_file:=./src/nav/mapper_params_online_sync.yaml use_sim_time:=False &')
    
    # Sauvegarder map Slam
    #ros2 run nav2_map_server map_saver_cli


