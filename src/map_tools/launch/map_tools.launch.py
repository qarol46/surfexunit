import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)

    map_publisher_params = [params_file]

    # Если map_yaml передан в launch, переопределяем параметр из yaml-файла
    if map_yaml:
        map_publisher_params.append({'map_yaml': map_yaml})

    map_publisher_node = Node(
        package='map_tools',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
        parameters=map_publisher_params,
    )

    inflation_publisher_node = Node(
        package='map_tools',
        executable='inflation_publisher',
        name='inflation_publisher',
        output='screen',
        parameters=[params_file],
    )

    zones_publisher_node = Node(
        package='map_tools',
        executable='zones_publisher',
        name='zones_publisher',
        output='screen',
        parameters=[params_file],
    )

    return [
        map_publisher_node,
        inflation_publisher_node,
        zones_publisher_node,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('map_tools')
    default_params = os.path.join(pkg_share, 'config', 'map_tools.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to parameter YAML file'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='',
            description='Full path to map.yaml, overrides parameter file'
        ),
        OpaqueFunction(function=build_nodes),
    ])