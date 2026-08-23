import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_asump_description = get_package_share_directory('asump_description')
    pkg_asump_bringup = get_package_share_directory('asump_bringup')
    
    urdf_file = os.path.join(pkg_asump_description, 'urdf', 'robot.xacro')
    twist_mux_params_file = os.path.join(pkg_asump_bringup, 'config', 'twist_mux.yaml')

    # Парсинг XACRO в строку URDF
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('asump_sim'), 'launch', 'launch_sim.launch.py')
        ),

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': robot_description}]
        # ),

        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('asump_localization'), 'launch', 'localization.launch.py')
        ),

        Node(
            package='map_tools',
            executable='path_planner',
            name='path_planner',
            output='screen'
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_params_file],
        ),

        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('asump_nav'), 'launch', 'navigation.launch.py')
        ),
    ])