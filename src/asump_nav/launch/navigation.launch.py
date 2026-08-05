import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('asump_nav')
    config = os.path.join(pkg_dir, 'config', 'path_follower.yaml')

    return LaunchDescription([
        Node(
            package='asump_nav',
            executable='path_follower_node',
            name='path_follower',
            output='screen',
            parameters=[config],
        ),
    ])