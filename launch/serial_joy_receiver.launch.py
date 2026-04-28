from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('serial_joy_receiver')

    config_file = os.path.join(
        pkg_share,
        'config',
        'serial_joy_receiver.yaml'
    )

    return LaunchDescription([
        # 🔹 시리얼 → /joy
        Node(
            package='serial_joy_receiver',
            executable='serial_joystick_receiver',
            name='serial_joystick_receiver',
            output='screen',
            parameters=[config_file]
        ),

        # 🔹 /joy → /cango_control_out
        Node(
            package='serial_joy_receiver',
            executable='joy_to_robot_control',
            name='joy_to_robot_control',
            output='screen',
            parameters=[config_file]
        ),

        # 🔥 추가: vibration 제어 노드
        Node(
            package='serial_joy_receiver',
            executable='serial_trigger_sender',
            name='serial_trigger_sender',
            output='screen',
            parameters=[config_file]
        )
    ])