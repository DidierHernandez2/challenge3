from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_trayectoria',
            executable='pid_control',
            name='pid_control_node',
            output='screen'
        )
    ])
