import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    use_rear_str = LaunchConfiguration('use_rear_camera').perform(context)
    use_rear_camera = use_rear_str.lower() == 'true'

    asump_localization_dir = get_package_share_directory('asump_localization')

    rviz_config_file = os.path.join(asump_localization_dir, 'config', 'demo_robot_mapping.rviz')

    rtabmap_params_file = os.path.join(
        asump_localization_dir, 'config',
        'rtabmap_params.yaml' if use_rear_camera else 'rtabmap_params_1cam.yaml'
    )

    # Переопределения поверх YAML (имеют приоритет)
    overrides = {
        'use_sim_time': True,
    }

    nodes = []

    # --- Синхронизация RGB-D потока передней камеры ---
    rgbd_sync_front = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync_front',
        output='screen',
        parameters=[
            rtabmap_params_file,
            overrides,
            {'approx_sync_max_interval': 0.05},
        ],
        remappings=[
            ('rgb/image', '/face_camera/image'),
            ('depth/image', '/face_camera/depth_image'),
            ('rgb/camera_info', '/face_camera/camera_info'),
            ('rgbd_image', '/front_camera/rgbd_image'),
        ],
    )
    nodes.append(rgbd_sync_front)

    # --- Облако препятствий передней камеры (для obstacle_layer Nav2) ---
    nodes.append(Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='point_cloud_xyzrgb_front',
        output='screen',
        parameters=[rtabmap_params_file, overrides],
        remappings=[
            ('rgb/image', '/face_camera/image'),
            ('depth/image', '/face_camera/depth_image'),
            ('rgb/camera_info', '/face_camera/camera_info'),
            ('cloud', '/front_camera/voxel_cloud'),
        ],
    ))
    nodes.append(Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection_front',
        output='screen',
        parameters=[rtabmap_params_file, overrides],
        remappings=[
            ('cloud', '/front_camera/voxel_cloud'),
            ('obstacles', '/front_camera/obstacles'),
            ('ground', '/front_camera/ground'),
        ],
    ))

    # --- Задняя камера (опционально) ---
    if use_rear_camera:
        nodes.append(Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_rear',
            output='screen',
            parameters=[
                rtabmap_params_file,
                overrides,
                {'approx_sync_max_interval': 0.05},
            ],
            remappings=[
                ('rgb/image', '/back_camera/image'),
                ('depth/image', '/back_camera/depth_image'),
                ('rgb/camera_info', '/back_camera/camera_info'),
                ('rgbd_image', '/rear_camera/rgbd_image'),
            ],
        ))
        nodes.append(Node(
            package='rtabmap_util',
            executable='point_cloud_xyzrgb',
            name='point_cloud_xyzrgb_rear',
            output='screen',
            parameters=[rtabmap_params_file, overrides],
            remappings=[
                ('rgb/image', '/back_camera/image'),
                ('depth/image', '/back_camera/depth_image'),
                ('rgb/camera_info', '/back_camera/camera_info'),
                ('cloud', '/rear_camera/voxel_cloud'),
            ],
        ))
        nodes.append(Node(
            package='rtabmap_util',
            executable='obstacles_detection',
            name='obstacles_detection_rear',
            output='screen',
            parameters=[rtabmap_params_file, overrides],
            remappings=[
                ('cloud', '/rear_camera/voxel_cloud'),
                ('obstacles', '/rear_camera/obstacles'),
                ('ground', '/rear_camera/ground'),
            ],
        ))

        # --- Склейка RGB-D потоков обеих камер в одно сообщение RGBDImages ---
        # Бинарный rtabmap_ros собран без RTABMAP_SYNC_MULTI_RGBD, поэтому
        # нода rtabmap не умеет синхронизировать 2 топика rgbd_image
        # (rgbd_cameras=2 -> FATAL). rgbdx_sync синхронизирует их сам и
        # публикует единый топик rgbd_images (принимается при rgbd_cameras=0).
        # ВАЖНО: общий YAML сюда не передаём — в нём rgbd_cameras=0, а этой
        # ноде нужно ровно 2 входа.
        nodes.append(Node(
            package='rtabmap_sync',
            executable='rgbdx_sync',
            name='rgbdx_sync',
            output='screen',
            parameters=[
                overrides,
                {
                    'rgbd_cameras': 2,
                    'approx_sync': True,
                    'sync_queue_size': 10,
                },
            ],
            remappings=[
                ('rgbd_image0', '/front_camera/rgbd_image'),
                ('rgbd_image1', '/rear_camera/rgbd_image'),
                ('rgbd_images', '/rgbd_images'),
            ],
        ))

    # --- Одометрия по лидару ---
    nodes.append(Node(
        package='asump_localization',
        executable='lidar_odometry_node',
        name='lidar_odometry_node',
        output='screen',
        parameters=[{
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
        }],
    ))

    # --- SLAM ---
    # Две камеры: rtabmap подписан на единый топик rgbd_images
    # (rtabmap_msgs/RGBDImages от rgbdx_sync, rgbd_cameras=0 в YAML).
    # Одна камера: обычный топик rgbd_image (rgbd_cameras=1).
    # ВАЖНО: для каждой камеры должен быть точный TF base_footprint ->
    # оптический фрейм камеры.
    if use_rear_camera:
        rtabmap_remappings = [
            ('rgbd_images', '/rgbd_images'),
        ]
    else:
        rtabmap_remappings = [
            ('rgbd_image', '/front_camera/rgbd_image'),
        ]
    rtabmap_remappings += [
        ('scan', '/scan'),
        ('odom', '/lidar_odometry'),
    ]

    nodes.append(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_params_file, overrides],
        remappings=rtabmap_remappings,
        arguments=['-d'],
    ))

    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'use_rear_camera',
            default_value='true',
            description='Use rear-facing depth camera if true (2-camera SLAM)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
