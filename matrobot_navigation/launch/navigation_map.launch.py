#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ==================================================
    # Package paths
    # ==================================================
    pkg_nav = get_package_share_directory('matrobot_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # ==================================================
    # Launch arguments
    # ==================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_nav,
            'maps',
            'map_1768051123',
            'map_1768051123.yaml'
        ),
        description='Full path to map YAML file'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            pkg_nav,
            'config',
            'nav2_params.yaml'
        ),
        description='Full path to Nav2 parameter file'
    )

    # ==================================================
    # Localization (map_server + localization)
    # ==================================================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2,
                'launch',
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # ==================================================
    # Navigation core (planner, controller, BT)
    # ==================================================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # ==================================================
    # RViz 
    # ==================================================
    
    rviz_config = os.path.join(
        get_package_share_directory('matrobot_navigation'),
        'rviz',
        'navigation_map.rviz'
    )
    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # ==================================================
    # Launch description
    # ==================================================
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_map,
        declare_params_file,
        localization_launch,
        navigation_launch,
        rviz_launch
    ])

