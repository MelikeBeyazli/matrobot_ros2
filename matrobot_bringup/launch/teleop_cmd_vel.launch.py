from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('matrobot_bringup'),
        'config',
        'xbox_cmd_vel.yaml'
    ])

    return LaunchDescription([
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[config_file],
            output='screen'
        )
    ])
