cmake_minimum_required(VERSION 3.5)
project(robot_nav)

# Trouver les dependances systeme
find_package(ament_cmake REQUIRED)

# Installer les fichiers de lancement
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Installer les fichiers de carte
install(DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

# Installer les fichiers de configuration
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
