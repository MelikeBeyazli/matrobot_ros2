#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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
    # Package paths
    # ========================
    sim_pkg  = get_package_share_directory('matrobot_simulation')

    world_path = os.path.join(
        sim_pkg,
        'worlds',
        'obstacles_world.world'
    )

    bridge_config = os.path.join(
        sim_pkg,
        'config',
        'gz_bridge.yaml'
    )

    # ==================================================
    # Robot Description (URDF + TF + RViz)
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

    # ========================
    # Gazebo (Ignition / GZ)
    # ========================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v1 {world_path}'
        }.items()
    )

    # ========================
    # Spawn robot
    # ========================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'matrobot',
            '-x', '4.36',
            '-y', '-2.23',
            '-Y', '3.14',
        ],
    )

    # ========================
    # ROS <-> GZ bridge
    # ========================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ]
    )
    # ==================================================
    # Localization (EKF)
    # ==================================================
    # ========================
    # EKF config path
    # ========================
    ekf_config = PathJoinSubstitution([
        sim_pkg,
        'config',
        'ekf.yaml'
    ])

    # ========================
    # Local EKF (odom frame)
    # ========================
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, ekf_config],
        remappings=[('odometry/filtered', '/odometry/local')]
    )


    return LaunchDescription([
        declare_sim_time,
        declare_rviz,

        gazebo,
        description_launch,
        spawn_robot,
        bridge,
        ekf_local,
    ])
