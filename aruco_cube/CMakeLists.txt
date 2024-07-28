cmake_minimum_required(VERSION 3.5)
project(aruco_cube)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

# Include directories
include_directories(include)

# Install URDF and meshes
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.xacro" PATTERN "*.dae")

# Install textures
install(DIRECTORY Aruco_tags
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.png")

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  codes/update_dae.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

