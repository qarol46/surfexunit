import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)

    # ==================== map_server (замена map_publisher) ====================
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'yaml_filename': map_yaml,
    #         # Переименовываем топик, чтобы соответствовать вашей системе
    #         'topic_name': 'map_base',
    #         'frame_id': 'map',
    #     }],
    # )

    # ==================== Lifecycle manager ====================
    # Автоматически переводит map_server в ACTIVE состояние при запуске
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ==================== Остальные ноды map_tools ====================
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
        #map_server_node,
        #lifecycle_manager_node,
        inflation_publisher_node,
        zones_publisher_node,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('map_tools')
    default_params = os.path.join(pkg_share, 'config', 'map_tools.yaml')
    
    # Путь к карте по умолчанию (можно переопределить через launch-аргумент)
    default_map_yaml = "/home/kirill/ros2_ws/src/surfexunit_ws/src/asump_localization/maps/new_warehouse_map.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to parameter YAML file'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description='Full path to map.yaml file'
        ),
        OpaqueFunction(function=build_nodes),
    ])