import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('asump_nav')
    config = os.path.join(pkg_dir, 'config', 'path_follower.yaml')

    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav.rviz')
    return LaunchDescription([
        Node(
            package='asump_nav',
            executable='path_follower_node',
            name='path_follower',
            output='screen',
            parameters=[config],
        ),

        Node(
            package='asump_nav',
            executable='mission_planner_node',
            name='mission_planner',
            output='screen',
        ),

        Node(
            package='asump_nav',
            executable='obstacle_safety_gate_node',
            name='obstacle_safety',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        ),
    ])