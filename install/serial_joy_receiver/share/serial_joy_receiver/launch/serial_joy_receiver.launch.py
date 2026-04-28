from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('serial_joy_receiver'),
        'config',
        'serial_joy_receiver.yaml'
    )

    return LaunchDescription([
        Node(
            package='serial_joy_receiver',
            executable='serial_joystick_receiver',
            name='serial_joystick_receiver',
            output='screen',
            parameters=[config_file]
        )
    ])
