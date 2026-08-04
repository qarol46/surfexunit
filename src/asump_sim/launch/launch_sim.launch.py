import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def _gazebo_resource_path(context):
    """Собирает пути к моделям для Gazebo"""
    gazebo_share = get_package_share_directory('asump_sim')
    bundled_paths = [
        os.path.join(gazebo_share, 'models'),
        gazebo_share,
    ]
    
    # Добавляем путь к описанию робота (для mesh файлов)
    try:
        desc_share = get_package_share_directory('asump_description')
        bundled_paths.append(desc_share)
    except Exception:
        pass
    
    existing_paths = [p for p in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(os.pathsep) if p]
    
    # Убираем дубликаты
    all_paths = []
    seen = set()
    for path in bundled_paths + existing_paths:
        if path and path not in seen:
            all_paths.append(path)
            seen.add(path)
    
    return os.pathsep.join(all_paths)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    use_rviz_arg = LaunchConfiguration('use_rviz', default='true')

    asump_sim_pkg = get_package_share_directory('asump_sim')
    asump_description_pkg = get_package_share_directory('asump_description')
    
    world_path = os.path.join(asump_sim_pkg, 'worlds', 'warehouse.sdf')
    rviz_config = os.path.join(asump_sim_pkg, 'config', 'default.rviz')
    
    rsp_launch_file = os.path.join(asump_description_pkg, 'launch', 'rsp.launch.py')
    controllers_yaml = os.path.join(asump_description_pkg, 'config', 'controllers.yaml')
    
    # Устанавливаем путь к моделям для Gazebo
    gz_resource_path = OpaqueFunction(
        function=lambda context: [
            SetEnvironmentVariable(
                name='GZ_SIM_RESOURCE_PATH',
                value=_gazebo_resource_path(context),
            )
        ],
    )
    
    # RSP - публикация описания робота
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_launch_file]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': 'true'
        }.items()
    )
    
    # Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': world_path + ' -r',  # -r = auto-start
        }.items()
    )
    
    # Спавн робота
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'asump0',
            '-x', '2.5',
            '-y', '0.0',
            '-z', '0.8',
            '-allow_renaming', 'false'
        ],
        output='screen'
    )
    
    # Load before diff_drive_controller so /joint_states is available for wheel positions.
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=[
            "joint_broad",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="diff_drive_controller_spawner",
        arguments=[
            "diff_drive",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    # Мост ROS <-> Gazebo через YAML конфигурацию
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(asump_sim_pkg, 'config', 'ros_gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )
    
    delayed_control_spawner = TimerAction(
        period=6.0,  # Увеличенная задержка для надежной инициализации
        actions=[joint_broad_spawner, diff_drive_spawner]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        #condition=IfCondition(use_rviz_arg),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Использовать симулированное время'),
        DeclareLaunchArgument('use_rviz_arg', default_value='true',
                              description='Использовать RViz'),
        
        gz_resource_path,      # Пути к моделям
        rsp,                   # Описание робота
        gazebo,                # Gazebo Harmonic
        spawn_entity,          # Спавн робота
        #delayed_control_spawner,  # Контроллеры с задержкой
        ros_gz_bridge,         # Мост ROS <-> Gazebo
        rviz_node,             # RViz
    ])