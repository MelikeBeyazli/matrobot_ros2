cmake_minimum_required(VERSION 3.5)

# ==================================================
# Project Definition
# ==================================================
project(matrobot_bringup)

# ==================================================
# Required ROS 2 build system
# ==================================================
find_package(ament_cmake REQUIRED)

# ==================================================
# Install resource files (non-executable)
# These are typically used by launch files, RViz,
# simulation models, etc.
# ==================================================
install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

# ==================================================
# Install Python package files
# This allows importing matrobot_bringup as a module
# e.g. `import matrobot_bringup`
# ==================================================
install(
  DIRECTORY matrobot_bringup/
  DESTINATION lib/site-packages/matrobot_bringup
  FILES_MATCHING PATTERN "*.py"
)

# ==================================================
# Install executable Python nodes
# These scripts will be callable via:
# ros2 run matrobot_bringup <executable_name>
# ==================================================

# --------------------------------------------------
# Python Node
# --------------------------------------------------
install(
  PROGRAMS
    matrobot_bringup/joy_multi_toggle.py
  DESTINATION
    lib/${PROJECT_NAME}
  RENAME joy_multi_toggle
)

install(
  PROGRAMS
    matrobot_bringup/keyboard_teleop_twist.py
  DESTINATION
    lib/${PROJECT_NAME}
  RENAME keyboard_teleop_twist
)

install(
  PROGRAMS
    matrobot_bringup/lidar_avoidance.py
  DESTINATION
    lib/${PROJECT_NAME}
  RENAME lidar_avoidance
)
# ==================================================
# Declare package to ROS 2 build system
# ==================================================
ament_package()



