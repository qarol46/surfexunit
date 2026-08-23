import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    asump_localization_dir = get_package_share_directory('asump_localization')
    
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    slam_config = os.path.join(asump_localization_dir, 'config', 'slam_localization_params.yaml')
    ekf_config = os.path.join(asump_localization_dir, 'config', 'ekf.yaml')
    amcl_config = os.path.join(asump_localization_dir, 'config', 'amcl.yaml')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value='/home/kirill/ros2_ws/src/surfexunit_ws/src/asump_localization/maps/new_warehouse_map.yaml',
        description='Full path to map yaml file'
    )

    # 1. Нода инициализации позы (ваш кастомный скан-матчер)
    scan_matcher_node = Node(
        package='asump_localization',
        executable='pose_initializer_node',
        name='scan_matcher_service',
        output='screen',
        parameters=[{
            'map_topic': '/map_base',
            'scan_topic': '/scan',
            'charger_pose_topic': '/zones/charger_pose',
            'output_pose_topic': '/initialpose',
            'voxel_size': 0.05,
            'map_occupied_threshold': 65,
            'yaw_search_range_deg': 45.0,
            'yaw_search_step_deg': 5.0,
        }],
    )

    # 2. Лидарная одометрия
    lidar_odometry_node = Node(
        package='asump_localization',
        executable='lidar_odometry_node',
        name='lidar_odometry_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. EKF (Комплексирование)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    # 4. AMCL (Глобальная локализация)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',  # <-- Имя ноды должно быть 'amcl'
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': True}
        ],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml_file,
            # Переименовываем топик, чтобы соответствовать вашей системе
            'topic_name': 'map_base',
            'frame_id': 'map',
        }],
    )

    # 5. ЕДИНЫЙ Lifecycle Manager для map_server и amcl
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'], 
        }],
    )

    # 6. Автоматический вызов инициализации позы через 5 секунд
    auto_init_service = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/initialize_pose',
                    'asump_localization/srv/InitializePose',
                    '{initial_guess: {header: {frame_id: ""}}, use_3d_refinement: false}'
                ],
                output='screen',
            )
        ]
    )

    # 7. Включаем map_tools (он уже содержит свой map_server, но мы его переопределим или объединим)
    # Примечание: Лучше, чтобы map_server запускался здесь, в этом же менеджере.
    # Если map_tools.launch.py тоже запускает map_server, нужно выбрать что-то одно.
    # Для простоты оставим вызов map_tools, но убедимся, что AMCL использует /map_base.

    map_tools_launch_include = IncludeLaunchDescription( # Не забудьте импортировать IncludeLaunchDescription
        PathJoinSubstitution([
            get_package_share_directory('map_tools'),
            'launch', 'map_tools.launch.py'
        ]),
        launch_arguments={'map_yaml': map_yaml_file}.items(),
    )

    return LaunchDescription([
        declare_map_yaml_cmd,
        lidar_odometry_node,
        ekf_node,
        scan_matcher_node,
        amcl_node,
        map_server_node,
        lifecycle_manager_node,
        map_tools_launch_include,
        auto_init_service,
    ])