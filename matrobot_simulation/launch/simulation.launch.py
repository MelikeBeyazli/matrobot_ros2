#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ========================
    # Launch arguments
    # ========================
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz     = LaunchConfiguration('use_rviz')
    ekf_yaml     = LaunchConfiguration('ekf_yaml')

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

    declare_ekf = DeclareLaunchArgument(
        'ekf_yaml', default_value='ekf_imu',
        description='EKF configuration file (without .yaml)'
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
            '-name', 'matrobot'
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
        ],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local'
        }]
    )
    # ==================================================
    # Localization (EKF)
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


    return LaunchDescription([
        declare_sim_time,
        declare_rviz,
        declare_ekf,

        gazebo,
        description_launch,
        spawn_robot,
        bridge,
        localization_launch,
    ])
