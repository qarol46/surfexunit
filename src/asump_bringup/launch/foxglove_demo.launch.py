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
    arg_foxglove_port = DeclareLaunchArgument(
        'foxglove_port', default_value='8765',
        description='Порт WebSocket-сервера Foxglove'
    )

    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    realsense_pkg = get_package_share_directory('realsense2_camera')
    foxglove_pkg = get_package_share_directory('foxglove_bridge')
    description_path = get_package_share_directory('asump_description')
    
    sllidar_launch = os.path.join(sllidar_pkg, 'launch', 'sllidar_a3_launch.py')
    realsense_launch = os.path.join(realsense_pkg, 'launch', 'rs_launch.py')
    foxglove_launch = os.path.join(foxglove_pkg, 'launch', 'foxglove_bridge_launch.xml')
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

    control_node = Node(
        package='asump_control',
        executable='velocity_forward_node',
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
            ('enable_pointcloud', 'false'),
            ('enable_aligned_depth_color', 'false'),
            ('color_width', '640'),
            ('color_height', '480'),
            ('color_fps', '30'),
            ('image_transport', 'compressed'),
            ('image_qos', 'BEST_EFFORT'),
            ('filters', ''),
            ('initial_reset', 'true'),
            ('wait_for_device_timeout', '10.0'),
        ],
    )

    throttle_camera_raw = Node(
        package='topic_tools',
        executable='throttle',
        name='throttle_camera_color_raw',
        output='screen',
        arguments=[
            'messages',
            '/camera/camera/color/image_raw',
            '15',
            '/camera/camera/color/image_raw_throttled',
        ],
        parameters=[{'use_sim_time': False}],
    )

    republish_compressed = Node(
        package='image_transport',
        executable='republish',
        name='republish_camera_compressed',
        output='screen',
        arguments=[
            'raw',
            'compressed',
            '--in-topic', '/camera/camera/color/image_raw_throttled',
            '--out-topic', '/camera/camera/color/image_compressed',
        ],
        parameters=[{
            'use_sim_time': False,
            'compressed_format': 'jpeg',
            'jpeg_quality': 50,
        }],
    )

    include_foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch),
        launch_arguments=[
            ('port', LaunchConfiguration('foxglove_port')),
            ('address', '0.0.0.0'),
            ('topic_regex', '/cmd_vel|/joint_states|/scan|/camera/camera/color/image_compressed|/tf|/tf_static|/robot_description'),
            ('max_qos_depth', '3'),
            ('use_compression', 'true'),
            ('max_update_rate', '20.0'),
            ('capabilities', '["clientPublish","services","assets"]'),
            ('send_buffer_limit', '10000000'),
        ],
    )

    return LaunchDescription([
        arg_lidar_port,
        arg_foxglove_port,
        node_robot_state_publisher,
        control_node,
        imu_node,
        include_lidar,
        include_realsense,
        #throttle_camera_raw,
        #republish_compressed,
        #include_foxglove,
    ])