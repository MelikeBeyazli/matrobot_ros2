cmake_minimum_required(VERSION 3.5)

# ==================================================
# Project Definition
# ==================================================
project(matrobot_hardware)

# ==================================================
# ROS 2 build system and Python support
# ==================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ==================================================
# Runtime dependencies
# ==================================================
find_package(rclpy REQUIRED)

# ==================================================
# Install Python package files
# Enables: import matrobot_hardware
# ==================================================
install(
  DIRECTORY matrobot_hardware/
  DESTINATION lib/site-packages/matrobot_hardware
  FILES_MATCHING PATTERN "*.py"
)

# ==================================================
# Install executable Python node
# Callable via:
#   ros2 run matrobot_hardware HardwareNode
# ==================================================

# --------------------------------------------------
# Hardware Interface Node
# Handles low-level communication with robot hardware
# --------------------------------------------------
install(
  PROGRAMS
    matrobot_hardware/HardwareNode.py
  DESTINATION
    lib/${PROJECT_NAME}
  RENAME
    HardwareNode
)

# ==================================================
# Install configuration and launch files
# ==================================================
install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

# ==================================================
# Declare package to ROS 2 build system
# ==================================================
ament_package()

