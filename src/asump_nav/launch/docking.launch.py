import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('asump_nav'), 'config', 'docking.yaml')

    return LaunchDescription([
        Node(
            package='asump_nav',
            executable='dock_controller_node',
            name='dock_controller',
            output='screen',
            parameters=[params],
        ),
    ])
