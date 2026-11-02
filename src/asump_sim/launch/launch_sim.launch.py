import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import IfElseSubstitution

def _gazebo_resource_path(context):
    """Собирает пути к моделям для Gazebo"""
    gazebo_share = get_package_share_directory('asump_sim')
    bundled_paths = [
        os.path.join(gazebo_share, 'models'),
        gazebo_share,
    ]
    
    # Добавляем путь к описанию робота (для mesh файлов)
    try:
        desc_share = get_package_share_directory('asump_description2')
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

    use_rviz_arg = LaunchConfiguration('use_rviz_arg', default='false')
    use_foxglove = LaunchConfiguration('use_foxglove', default='true')
    headless = LaunchConfiguration('headless', default='true')

    asump_sim_pkg = get_package_share_directory('asump_sim')
    asump_description_pkg = get_package_share_directory('asump_description2')
    
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
    
    # Gazebo Harmonic.
    # headless=true — только сервер (gz sim -s), без GUI: в Docker-контейнере
    # GUI падает из-за отсутствия OpenGL 3.3+ через XQuartz, визуализация — через RViz.
    # DISPLAY для сервера очищаем: иначе ogre2 (нужен для камер/сенсоров) пытается
    # рендерить через GLX -> XQuartz и падает с GLXBadFBConfig. Без DISPLAY
    # рендеринг идёт через EGL + llvmpipe (программный, полностью headless).
    # SetEnvironmentVariable внутри GroupAction действует только на эту группу,
    # RViz и другие ноды DISPLAY по-прежнему получают.
    host_display = os.environ.get('DISPLAY', '')
    gazebo = GroupAction([
        SetEnvironmentVariable(
            name='DISPLAY',
            value=IfElseSubstitution(headless, '', host_display),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ]),
            launch_arguments={
                'gz_args': IfElseSubstitution(
                    headless,
                    world_path + ' -r -s',   # -s = server only, без GUI
                    world_path + ' -r',      # -r = auto-start
                ),
            }.items()
        ),
    ])
    
    # Спавн робота
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'asump0',
            '-x', '2.85',
            '-y', '-0.2',
            '-z', '0.3',
            '-Y', '1.2',
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
        condition=IfCondition(use_rviz_arg),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Foxglove Bridge — визуализация через Foxglove Studio на хосте.
    # В Docker-контейнере RViz не может создать GLX-контекст через XQuartz,
    # поэтому основной способ смотреть сцену — Foxglove (websocket :8765).
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(use_foxglove),
        parameters=[{
            'port': 8765,
            'use_sim_time': use_sim_time,
            # URDF робота ссылается на меши через file:///workspace/...
            # (так требует gz sim). Разрешаем мосту отдавать такие файлы
            # в Foxglove Studio, иначе модель робота не отобразится.
            'asset_uri_allowlist': [
                r'^package://(?:[\w]+/)*[\w]+\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$',
                r'^file:///workspace/.*\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp)$',
            ],
        }],
    )

    # Статические трансформы URDF-линк -> gz-фрейм сенсора (identity).
    # Gazebo публикует данные сенсоров в scoped-фреймах вида
    # "asump0/base_footprint/<sensor>", которых нет в TF-дереве от
    # robot_state_publisher. Соединяем оба дерева, чтобы данные сенсоров
    # отображались вместе с моделью робота.
    gz_sensor_frames = {
        'laser': 'asump0/base_footprint/lidar_sensor',
        'face_camera_color_frame': 'asump0/base_footprint/face_camera_color',
        'face_camera_depth_frame': 'asump0/base_footprint/face_camera_depth',
        'back_camera_color_frame': 'asump0/base_footprint/back_camera_color',
        'back_camera_depth_frame': 'asump0/base_footprint/back_camera_depth',
        'cam_left_link': 'asump0/base_footprint/cam_left_sensor',
        'cam_right_link': 'asump0/base_footprint/cam_right_sensor',
        'face_left_us_sensor_link': 'asump0/base_footprint/face_left_us_sensor',
        'face_right_us_sensor_link': 'asump0/base_footprint/face_right_us_sensor',
        'left1_us_sensor_link': 'asump0/base_footprint/left1_us_sensor',
        'left2_us_sensor_link': 'asump0/base_footprint/left2_us_sensor',
        'right1_us_sensor_link': 'asump0/base_footprint/right1_us_sensor',
        'right2_us_sensor_link': 'asump0/base_footprint/right2_us_sensor',
        'back_left_us_sensor_link': 'asump0/base_footprint/back_left_us_sensor',
        'back_right_us_sensor_link': 'asump0/base_footprint/back_right_us_sensor',
    }
    gz_frame_links = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'gz_link_{urdf_frame}',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', urdf_frame,
                '--child-frame-id', gz_frame,
            ],
        )
        for urdf_frame, gz_frame in gz_sensor_frames.items()
    ]

    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Использовать симулированное время'),
        DeclareLaunchArgument('use_rviz_arg', default_value='false',
                              description='Использовать RViz (требует рабочий OpenGL/GLX; в контейнере через XQuartz не работает)'),
        DeclareLaunchArgument('use_foxglove', default_value='true',
                              description='Запустить foxglove_bridge на порту 8765'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='Запуск Gazebo без GUI (только сервер)'),
        
        gz_resource_path,      # Пути к моделям
        rsp,                   # Описание робота
        gazebo,                # Gazebo Harmonic
        spawn_entity,          # Спавн робота
        #delayed_control_spawner,  # Контроллеры с задержкой
        ros_gz_bridge,         # Мост ROS <-> Gazebo
        rviz_node,             # RViz
        foxglove_node,         # Foxglove Bridge
    ])