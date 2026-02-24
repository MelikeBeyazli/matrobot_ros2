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
# install(
#   PROGRAMS
#     matrobot_bringup/NodeFile/Node.py
#   DESTINATION
#     lib/${PROJECT_NAME}
# )

# ==================================================
# Declare package to ROS 2 build system
# ==================================================
ament_package()

