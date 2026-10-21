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
    
    urdf_file = os.path.join(pkg_asump_description, 'urdf', 'robot1.xacro')

    # Парсинг XACRO в строку URDF
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='imu',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),     

        Node(
            package='asump_control',
            executable='asump_control_node',
            name='asump_control',
            output='screen'
        ),

        Node(
            package='asump_control',
            executable='lidar_transform_node',
            name='lidar_transform',
            output='screen'
        ),

        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a3_launch.py')
        ),

        IncludeLaunchDescription(os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'),
            launch_arguments=[
                ('enable_color', 'true'),
                ('enable_depth', 'true'),
                ('enable_infra1', 'false'),
                ('enable_infra2', 'false'),
                ('pointcloud.enable', 'false'),
            ],
        ),

        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('asump_localization'), 'launch', 'mapper.launch.py')
        ),
        
        # IncludeLaunchDescription(
        #     os.path.join(get_package_share_directory('map_tools'), 'launch', 'map_tools.launch.py')
        # ),
    ])