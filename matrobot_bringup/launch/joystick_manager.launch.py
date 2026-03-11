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
            package='joy',
            executable='game_controller_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen'
        ),

        Node(
            package='matrobot_bringup',
            executable='joy_multi_toggle',
            name='joy_multi_toggle',
            parameters=[{
                'button_a_index': 0,
                'button_b_index': 1,
                'button_x_index': 2,
                'button_y_index': 3,
                'debounce_sec': 0.35,

                'command_a': 'ros2 launch matrobot_bringup bringup.launch.py ekf_yaml:=ekf_imu_odom use_rviz:=false',
                'command_b': 'ros2 launch matrobot_slam slam_async.launch.py use_rviz:=false',
                'command_y': 'ros2 launch matrobot_simulation simulation.launch.py ekf_yaml:=ekf_imu_odom use_rviz:=false',

                'map_save_root': '~/matro_ws/src/matrobot_ros2/matrobot_navigation/maps',
                'map_save_command': 'ros2 run nav2_map_server map_saver_cli',
            }],
            output='screen'
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[config_file],
            output='screen'
        )
    ])
