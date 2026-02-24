#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # ========================
    # Package paths
    # ========================
    hardware_pkg = get_package_share_directory('matrobot_hardware')

    # ========================
    # Launch arguments
    # ========================
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_yaml     = LaunchConfiguration('ekf_yaml')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_ekf_yaml = DeclareLaunchArgument(
        'ekf_yaml',
        default_value='ekf_imu_odom',
        description='EKF YAML config name (without .yaml)'
    )

    # ========================
    # EKF config path
    # ========================
    ekf_config = PathJoinSubstitution([
        hardware_pkg,
        'config',
        PythonExpression(["'", ekf_yaml, ".yaml'"])
    ])

    # ========================
    # Local EKF (odom frame)
    # ========================
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, ekf_config],
        remappings=[('odometry/filtered', '/odometry/local')]
    )

    return LaunchDescription([
        declare_sim_time,
        declare_ekf_yaml,
        ekf_local,
    ])

