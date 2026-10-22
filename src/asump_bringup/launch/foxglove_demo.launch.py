from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os


def generate_launch_description():
    arg_lidar_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyLidar',
        description='Порт подключения лидара'
    )

    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    realsense_pkg = get_package_share_directory('realsense2_camera')
    description_path = get_package_share_directory('asump_description')
    
    sllidar_launch = os.path.join(sllidar_pkg, 'launch', 'sllidar_a3_launch.py')
    realsense_launch = os.path.join(realsense_pkg, 'launch', 'rs_launch.py')
    xacro_file = os.path.join(description_path, 'urdf', 'robot1.xacro')

    robot_description_config = Command([
        'xacro ', xacro_file, 
    ])
    
    robot_description = ParameterValue(robot_description_config, value_type=str)
    params = {
        'robot_description': robot_description, 
    }
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    web_teleop_node = Node(
        package='asump_web_teleop',
        executable='web_teleop_node',
        name='web_teleop',
        output='screen'
    )

    control_node = Node(
        package='asump_control',
        executable='asump_control_node',
        output='screen'
    )

    lidar_transform_node = Node(
        package='asump_control',
        executable='lidar_transform_node',
        output='screen'
    )

    imu_node = Node(
        package='imu',
        executable='imu_node',
        output='screen'
    )

    include_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch),
        launch_arguments=[
            ('serial_port', LaunchConfiguration('serial_port')),
        ],
    )

    include_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments=[
            ('enable_color', 'true'),
            ('enable_depth', 'true'),
            ('enable_infra1', 'false'),
            ('enable_infra2', 'false'),
            ('pointcloud.enable', 'false'),
        ],
    )

    return LaunchDescription([
        arg_lidar_port,
        node_robot_state_publisher,
        control_node,
        lidar_transform_node,
        web_teleop_node,
        imu_node,
        include_lidar,
        include_realsense,
    ])