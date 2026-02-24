cmake_minimum_required(VERSION 3.5)

# ==================================================
# Project Definition
# ==================================================
project(matrobot_description)

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
    rviz
    launch
    urdf
    meshes
  DESTINATION
    share/${PROJECT_NAME}
)

# ==================================================
# Declare package to ROS 2 build system
# ==================================================
ament_package()

