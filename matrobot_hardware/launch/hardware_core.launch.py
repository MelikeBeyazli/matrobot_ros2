#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ==================================================
    # Paths
    # ==================================================
    pkg_share = get_package_share_directory('matrobot_hardware')
    hardware_config = os.path.join(pkg_share, 'config', 'hardware_params.yaml')

    # ==================================================
    # Launch arguments
    # ==================================================
    serial_port  = LaunchConfiguration('serial_port')
    lidar_port   = LaunchConfiguration('lidar_port')  
    baudrate = LaunchConfiguration('baudrate')
    publish_tf = LaunchConfiguration('publish_tf')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyArduino'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyLIDAR'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('baudrate', default_value='115200'),

    ]

    # ==================================================
    # Hardware Interface Node
    # ==================================================
    hardware_node = Node(
        package='matrobot_hardware',
        executable='HardwareNode',
        name='matrobot_hardware_interface',
        output='screen',
        emulate_tty=True,
        parameters=[
            hardware_config,
            {
                'serial_port': serial_port,
                'baudrate': baudrate,
                'publish_tf': publish_tf,
            }
        ]
    )

    # ==================================================
    # IMU Driver
    # ==================================================
    imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('witmotion_hwt905_driver'),
                'launch',
                'imu.launch.py'
            )
        )
    )

    # ==================================================
    # 2D LiDAR Driver
    # ==================================================
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': 1000000,
            'frame_id': 'lidar2d_link',
            'angle_compensate': True,
            'scan_mode': 'DenseBoost'
            'inverted':True,
        }]
    )

    # ==================================================
    # Launch description
    # ==================================================
    return LaunchDescription([
        *declare_args,
        hardware_node,
        imu_driver,
        lidar_node,
    ])

