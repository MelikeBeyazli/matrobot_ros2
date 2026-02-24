#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ========================
    # Package path
    # ========================
    desc_pkg = get_package_share_directory('matrobot_description')

    # ========================
    # Launch arguments
    # ========================
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz     = LaunchConfiguration('use_rviz')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    # ========================
    # Robot description (Xacro)
    # ========================
    xacro_path = os.path.join(
        desc_pkg,
        'urdf',
        'matrobot.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    # ========================
    # Robot State Publisher
    # ========================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description}
        ],
    )

    # ========================
    # Joint State Publisher Gui
    # ========================
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ========================
    # RViz
    # ========================
    rviz_config = os.path.join(
        desc_pkg,
        'rviz',
        'matrobot.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_sim_time,
        declare_rviz,
        robot_state_publisher,
        rviz,
    ])

