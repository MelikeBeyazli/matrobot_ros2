from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
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
                'command_a': 'ros2 launch matrobot_bringup bringup.launch.py ekf_yaml:=ekf_imu_odom',
                'command_b': 'ros2 launch matrobot_bringup teleop_cmd_vel.launch.py',
                'command_x': 'ros2 launch matrobot_slam slam_async.launch.py use_rviz:=true',
                'map_save_root': '~/matrobot_ws/src/matrobot_ros2/matrobot_navigation/maps',
                'map_save_command': 'ros2 run nav2_map_server map_saver_cli',
            }],
            output='screen'
        )
    ])