cmake_minimum_required(VERSION 3.8)

# ==================================================
# Project Definition
# ==================================================
project(matrobot_simulation)

# ==================================================
# Compiler warning flags (optional but recommended)
# Enables stricter checks for C++ code if present
# ==================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================================================
# ROS 2 build system
# ==================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ==================================================
# Runtime dependencies (Python / ROS 2 interfaces)
# ==================================================
find_package(rclpy REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ==================================================
# Install resource files
# These are used by launch files, visualization,
# simulation, and configuration
# ==================================================
install(
  DIRECTORY
    config
    worlds
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

# ==================================================
# Install Python package files
# Allows: import matrobot_simulation
# ==================================================
install(
  DIRECTORY kangal_unity_ros/
  DESTINATION lib/site-packages/matrobot_simulation
  FILES_MATCHING PATTERN "*.py"
)

# ==================================================
# Declare package to ROS 2 build system
# ==================================================
ament_package()

