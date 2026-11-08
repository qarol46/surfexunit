import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_scan': True,
        'subscribe_rgb': True,
        'use_sim_time': use_sim_time,
        'approx_sync': True,
        'queue_size': 20,
        
        # --- НАСТРОЙКИ 2D КАРТЫ С ОБЪЕДИНЕНИЕМ ЛИДАРА И КАМЕРЫ ---
        'Reg/Force3DoF': 'true',       # Одометрия строго 2D
        'Grid/3D': 'false',            # Строим 2D occupancy grid (а не 3D OctoMap)
        'Grid/FromDepth': 'true',      # КЛЮЧЕВОЙ ПАРАМЕТР: использовать данные глубины для 2D сетки
        'Grid/CellSize': '0.05',       # Размер ячейки 5 см (стандарт для Nav2)
        
        # Фильтрация по высоте (критично для 4-колесного робота)
        'Grid/MinGroundHeight': '-0.15', # Игнорировать точки ниже этого уровня (пол)
        'Grid/MaxObstacleHeight': '1.5', # Считать препятствием всё, что выше этого уровня
        
        # Дальность восприятия для сетки (в метрах)
        'Grid/RangeMax': '6.0',        # Для лидара
        'Grid/DepthMax': '5.0',        # Для камеры глубины
        
        # Улучшение качества слияния
        'Grid/NormalsSegmentation': 'false', # Отключаем, чтобы не удалять "наклонные" препятствия с камеры
        'Grid/MaxGroundAngle': '30.0',       # Допустимый угол наклона поверхности (в градусах)
        
        'RGBD/ProximityBySpace': 'true',
        'Rtabmap/DetectMarker': 'false',
    }]

    lidar_odometry_node = Node(
        package='asump_localization',
        executable='lidar_odometry_node',
        name='lidar_odometry_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Основной узел SLAM
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=parameters,
        remappings=[
            ('rgb/image', '/face_camera/image'),
            ('depth/image', '/face_camera/depth_image'),
            ('rgb/camera_info', '/face_camera/camera_info'),
            ('scan', '/scan'),
            ('odom', '/lidar_odometry'),  # Ваша одометрия
            ('imu', '/imu/data'),         # IMU данные
        ]
    )

    # Узел визуализации
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=parameters,
        remappings=[
            ('rgb/image', '/face_camera/image'),
            ('depth/image', '/face_camera/depth_image'),
            ('rgb/camera_info', '/face_camera/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu/data'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        lidar_odometry_node,
        rtabmap_node,
        #rtabmap_viz_node,
    ])