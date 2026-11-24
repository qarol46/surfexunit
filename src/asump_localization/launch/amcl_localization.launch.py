import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


# Локализация на готовой карте в процессе навигации (AMCL).
#
# Потоки данных:
#   /scan -> icp_odometry --------------> /odom_icp --+
#   камера -> rgbd_sync -> rgbd_odometry -> /odom_rgbd --+--> EKF --> /odometry/filtered
#   /imu/data ---------------------------------------------+    + TF odom -> base_footprint
#
#   AMCL (nav2_amcl) по /scan и статической карте /map следит за позой робота
#   и публикует поправку TF map -> odom (одометрию для AMCL даёт EKF через TF).
#
#   nav2_map_server отдаёт статическую карту в /map (для Nav2, AMCL и
#   pose_initializer). pose_initializer_node по сервису /initialize_pose
#   находит стартовую позу фильтром частиц и шлёт её в /initialpose,
#   на которую подписан AMCL.
#
# ВАЖНО: не запускать одновременно lidar_odometry_node — конфликт TF
# odom -> base_footprint (его теперь публикует EKF).
# Перед запуском должна работать симуляция/робот (scan, камеры, /clock)
# и robot_state_publisher (TF base_footprint -> laser/камеры).

def launch_setup(context, *args, **kwargs):
    use_sim_time = True
    auto_initialize = True
    map_yaml_file = LaunchConfiguration('map_yaml_file').perform(context)

    asump_localization_dir = get_package_share_directory('asump_localization')
    amcl_params_file = os.path.join(asump_localization_dir, 'config', 'amcl.yaml')
    ekf_params_file = os.path.join(asump_localization_dir, 'config', 'ekf_odom_fusion.yaml')
    rviz_config_file = os.path.join(asump_localization_dir, 'config', 'localization.rviz')

    overrides = {'use_sim_time': use_sim_time}
    nodes = []

    # --- Статическая карта (nav2_map_server + lifecycle-активация) ---
    nodes.append(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        }],
    ))

    # --- AMCL: поправка TF map -> odom по лидару и карте ---
    # Одометрию AMCL берёт из TF odom -> base_footprint (её публикует EKF,
    # т.е. AMCL работает на фьюзе lidar-ICP + RGB-D + IMU).
    # Стартовую позу получает из /initialpose (pose_initializer).
    nodes.append(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file, overrides],
    ))

    # --- Lifecycle-менеджер для map_server и amcl ---
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    ))

    # --- Синхронизация RGB-D передней камеры (вход для rgbd_odometry) ---
    nodes.append(Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync_front',
        output='screen',
        parameters=[overrides, {'approx_sync': True, 'approx_sync_max_interval': 0.02}],
        remappings=[
            ('rgb/image', '/face_camera/image'),
            ('depth/image', '/face_camera/depth_image'),
            ('rgb/camera_info', '/face_camera/camera_info'),
            ('rgbd_image', '/front_camera/rgbd_image'),
        ],
    ))

    # --- Одометрия ICP по 2D-лидару (без TF — его даёт EKF) ---
    nodes.append(Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': False,
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'expected_update_rate': 10.0,
            'wait_for_transform_duration': 0.2,
            'Reg/Strategy': '1',        # 1 = чистый ICP
            'Reg/Force3DoF': 'true',
            'Icp/PointToPlane': 'false',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/Iterations': '30',
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom_icp'),
        ],
    ))

    # --- Визуальная одометрия по depth-камере (без TF) ---
    nodes.append(Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': False,
            'subscribe_rgbd': True,
            'wait_for_transform_duration': 0.2,
            'Reg/Force3DoF': 'true',
            'Odom/Strategy': '0',       # 0 = Frame-to-Map (точнее на текстурах)
            'Vis/MinInliers': '15',
        }],
        remappings=[
            ('rgbd_image', '/front_camera/rgbd_image'),
            ('odom', '/odom_rgbd'),
        ],
    ))

    # --- Фьюз одометрии: EKF -> /odometry/filtered + TF odom -> base_footprint ---
    nodes.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, overrides],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    ))

    # --- Инициализация стартовой позы фильтром частиц ---
    # Публикует /initialpose (transient_local) — его подхватывает AMCL.
    # Карту берёт из /map (nav2_map_server), prior зарядной станции —
    # опционально из /zones/charger_pose (map_tools), без него — глобальный поиск.
    nodes.append(Node(
        package='asump_localization',
        executable='pose_initializer_node',
        name='pose_initializer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': '/map',
            'scan_topic': '/scan',
            'charger_pose_topic': '/zones/charger_pose',
            'output_pose_topic': '/initialpose',
        }],
    ))

    # Поза зарядной станции (/zones/charger_pose) как prior для pose_initializer
    nodes.append(Node(
        package='map_tools',
        executable='zones_publisher',
        name='zones_publisher',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('map_tools'), 'config', 'zones_publisher.yaml'),
            overrides,
        ],
    ))

    # Автовызов инициализации: ждём, пока lifecycle активирует map_server/amcl
    # и pose_initializer получит карту и первый скан.
    # init_area выбирает область инициализации: "charger" (у зарядки),
    # "stairs" (у лестницы) или "" (глобальный поиск по всей карте).
    if auto_initialize:
        nodes.append(TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/initialize_pose',
                        'asump_localization/srv/InitializePose',
                        ['{initial_guess: {header: {frame_id: "',
                          LaunchConfiguration('init_area'),
                          '"}}, use_3d_refinement: false}'],
                    ],
                    output='screen',
                )
            ],
        ))

    if LaunchConfiguration('use_rviz').perform(context).lower() == 'true':
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
        ))

    return nodes


def generate_launch_description():
    asump_localization_dir = get_package_share_directory('asump_localization')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=PathJoinSubstitution(
                [asump_localization_dir, 'maps', 'first_floor.yaml']),
            description='Occupancy map yaml for nav2_map_server'
        ),
        DeclareLaunchArgument(
            'auto_initialize',
            default_value='true',
            description='Automatically call /initialize_pose 8 s after startup'
        ),
        DeclareLaunchArgument(
            'init_area',
            default_value='charger',
            description='Area for particle filter init: "charger", "stairs" '
                        'or "" for global search'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with nav.rviz'
        ),
        OpaqueFunction(function=launch_setup)
    ])
