cmake_minimum_required(VERSION 3.5)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

# === Installation des fichiers de lancement ===
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# === Installation des fichiers de configuration ===
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

