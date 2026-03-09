#!/bin/bash

export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=22

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/mb/matro_ws/install/setup.bash

exec ros2 launch matrobot_bringup joystick_manager.launch.py
