#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ==================================================
    # Launch arguments (system-level)
    # ==================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz     = LaunchConfiguration('use_rviz')
    ekf_yaml     = LaunchConfiguration('ekf_yaml')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz visualization'
        ),
        DeclareLaunchArgument(
            'ekf_yaml', default_value='ekf_imu',
            description='EKF configuration file (without .yaml)'
        ),
    ]

    # ==================================================
    # Robot description (URDF + TF + RViz)
    # ==================================================
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('matrobot_description'),
                'launch',
                'description.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items()
    )

    # ==================================================
    # Hardware core (motors + sensors)
    # ==================================================
    hardware_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('matrobot_hardware'),
                'launch',
                'hardware_core.launch.py'
            )
        ),
    )

    # ==================================================
    # Localization (EKF + NavSat)
    # ==================================================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('matrobot_hardware'),
                'launch',
                'localization.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ekf_yaml': ekf_yaml,
        }.items()
    )

    # ==================================================
    # Launch description
    # ==================================================
    return LaunchDescription([
        *declare_args,
        description_launch,
        hardware_core_launch,
        localization_launch,
    ])

