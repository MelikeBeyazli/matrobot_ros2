#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    ROS 2 launch sistemi bu fonksiyonu çağırır ve burada tanımlanan
    tüm launch argümanları ile alt launch dosyalarını çalıştırır.
    """

    # ==================================================
    # 1) Launch argument'lerini LaunchConfiguration olarak oku
    # ==================================================
    # Bunlar, terminalden verilen değerleri çalışma anında kullanmamızı sağlar.
    # Örnek:
    # ros2 launch ... use_rviz:=false serial_port:=/dev/ttyUSB0
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    ekf_yaml = LaunchConfiguration('ekf_yaml')
    serial_port = LaunchConfiguration('serial_port')
    use_ekf = LaunchConfiguration('use_ekf')
    publish_tf = LaunchConfiguration('publish_tf')

    # ==================================================
    # 2) Kullanıcıya açık launch argümanlarını tanımla
    # ==================================================
    # Bunlar bu launch dosyasının dışarıdan alacağı parametrelerdir.
    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Simülasyon zamanı kullanılsın mı?'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='RViz açılsın mı?'
        ),
        DeclareLaunchArgument(
            'ekf_yaml',
            default_value='ekf_imu',
            description='Kullanılacak EKF ayar dosyasının adı (.yaml olmadan)'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Donanımın bağlı olduğu seri port'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='TF yayını yapılsın mı?'
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Localization / EKF launch dosyası çalıştırılsın mı?'
        ),
    ]

    # ==================================================
    # 3) Robot description launch dosyasını dahil et
    # ==================================================
    # matrobot_description paketindeki description.launch.py dosyasını çalıştırır.
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
    # 4) Donanım çekirdeği launch dosyasını dahil et
    # ==================================================
    # matrobot_hardware paketindeki hardware_core.launch.py dosyasını çalıştırır.
    hardware_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('matrobot_hardware'),
                'launch',
                'hardware_core.launch.py'
            )
        ),
        launch_arguments={
            'serial_port': serial_port,
            'publish_tf': publish_tf,
        }.items()
    )

    # ==================================================
    # 5) Localization launch dosyasını dahil et
    # ==================================================
    # Sadece use_ekf true ise çalışır.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('matrobot_hardware'),
                'launch',
                'localization.launch.py'
            )
        ),
        condition=IfCondition(use_ekf),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ekf_yaml': ekf_yaml,
        }.items()
    )

    # ==================================================
    # 6) Tüm parçaları bir LaunchDescription içinde döndür
    # ==================================================
    return LaunchDescription([
        *declare_args,
        description_launch,
        hardware_core_launch,
        localization_launch,
    ])
