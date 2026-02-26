cmake_minimum_required(VERSION 3.5)
project(matrobot_slam)

# ==================================================
# Required build system
# ==================================================
find_package(ament_cmake REQUIRED)

# ==================================================
# Install non-executable resources
# These files are used by launch files and tools
# ==================================================
install(
  DIRECTORY
    launch
    config
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

# ==================================================
# Declare package
# ==================================================
ament_package()

