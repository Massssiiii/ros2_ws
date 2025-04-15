# -*- coding: utf-8 -*-

import subprocess
import time

def run_command(command):
    """Execute une commande en tant que sous-processus."""
    try:
        subprocess.run(command, check=True, shell=True)
    except subprocess.CalledProcessError as e:
        print(f"Erreur lors de l'execution de la commande : {command}")
        print(e)

if __name__ == '__main__':
    # Lancer micro_ros_agent
    #run_command('screen -dmS myssesion1 ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 &')

    # Lancer rplidar
    run_command('screen -dmS mysession2 ros2 launch rplidar_ros rplidar_c1_launch.py &')
    
    # Lancer rplidar
    run_command('screen -dmS mysession3 ros2 launch laser_filters range_filter_example.launch.py params_file:="/home/rasp/ros2_ws/src/laser_filters/examples/range_filter_example.yaml"')
    
    # Lancer rf2o_laser_odometry
    run_command('screen -dmS mysession4 ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &')

    # Lancer la description du robot
    run_command('screen -dmS mysession5 ros2 launch robot_description launch.launch.py &')
    
    # test pour essayer de resoudre le probleme des time stamp
    run_command('screen -dmS mysession6 ros2 launch slam_toolbox online_async_launch.py params_file:=./src/nav/mapper_params_online_sync.yaml use_sim_time:=False &')
    
    #donner le temps à nav2 pour tout charger avant de lancer les commandes
    time.sleep(15)
    
    # test pour essayer de resoudre le probleme des time stamp
    run_command('screen -dmS mysession7 ros2 launch nav2_bringup bringup_launch.py map:=map_1741680568.yaml')
    
    # test pour essayer de resoudre le probleme des time stamp
    #run_command('screen -dmS mysession8 ros2 launch nav2_bringup rviz_launch.py')
    
    # Sauvegarder map Slam
    #ros2 run nav2_map_server map_saver_cli


