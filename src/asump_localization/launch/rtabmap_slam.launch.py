import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'

    localization = LaunchConfiguration('localization').perform(context)

    asump_localization_dir = get_package_share_directory('asump_localization')
    map_tools_dir = get_package_share_directory('map_tools')
    zones_publisher_yaml = os.path.join(map_tools_dir, 'config', 'zones_publisher.yaml')
    rviz_config_file = os.path.join(asump_localization_dir, 'config', 'demo_robot_mapping.rviz')

    map_file_arg = os.path.join(asump_localization_dir, 'maps', 'test_rtabmap.yaml')

    parameters = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'odom_tf_linear_variance': 0.001,
        'odom_tf_angular_variance': 0.001,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'approx_sync': True,
        'sync_queue_size': 10,
        'use_sim_time': use_sim_time,
        'publish_tf': True,
        
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityByTime': 'false',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Reg/Strategy': '2',
        'Vis/MinInliers': '12',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '4',
        'Reg/Force3DoF': 'true',
        'Grid/Sensor': '2',
        'Mem/STMSize': '30',
        'RGBD/LocalRadius': '5',
        'Icp/CorrespondenceRatio': '0.2',
        'Icp/Strategy': '0',
        'Icp/PointToPlane': 'false',
        'Icp/MaxCorrespondenceDistance': '0.15',
        'Icp/VoxelSize': '0.05',
    }
    
    remappings = [
        ('rgb/image', '/face_camera/image'),
        ('depth/image', '/face_camera/depth_image'),
        ('rgb/camera_info', '/face_camera/camera_info'),
        ('scan', '/scan'),
        ('odom', '/lidar_odometry'),
    ]
    
    return [
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            output='screen',
            parameters=[parameters, {
                'approx_sync_max_interval': 0.02,
                'use_sim_time': use_sim_time
            }],
            remappings=remappings
        ),
        
        Node(
            package='asump_localization',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_footprint'
            }]
        ),
        
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']
        ),
        
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[parameters, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True'
            }],
            remappings=remappings
        ),

        # Node(
        #     condition=IfCondition(localization),
        #     package='asump_localization',
        #     executable='pose_initializer_node',
        #     name='scan_matcher_service',
        #     output='screen',
        #     parameters=[{
        #         'map_topic': '/map_base',
        #         'scan_topic': '/scan',
        #         'charger_pose_topic': '/zones/charger_pose',
        #         'output_pose_topic': '/initialpose',
        #         'voxel_size': 0.05,
        #         'map_occupied_threshold': 65,
        #         'yaw_search_range_deg': 45.0,
        #         'yaw_search_step_deg': 5.0,
        #     }],
        # ),

        
        # # Визуализация
        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmap_viz',
        #     output='screen',
        #     parameters=[parameters],
        #     remappings=remappings
        # ),

            # 1. Нода map_server
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     parameters=[{
        #         'yaml_filename': map_file_arg,
        #         'use_sim_time': use_sim_time
        #     }],
        # ),

        # # 2. Lifecycle Manager для автоматической активации map_server
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'autostart': True,
        #         'node_names': ['map_server'] # Список нод, которыми управляет этот менеджер
        #     }],
        # ),

        Node(
            package='map_tools',
            executable='zones_publisher',
            name='zones_publisher',
            output='screen',
            parameters=[zones_publisher_yaml],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        ),

        # TimerAction(
        #     period=3.0,  # секунд после старта launch
        #     condition = IfCondition(localization),
        #     actions=[
        #         ExecuteProcess(
        #             cmd=[
        #                 'ros2', 'service', 'call',
        #                 '/initialize_pose',
        #                 'asump_localization/srv/InitializePose',
        #                 '{initial_guess: {header: {frame_id: ""}}, use_3d_refinement: false}'
        #             ],
        #             output='screen',
        #         )
        #     ]
        # )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='true',
            description='Launch in localization mode'
        ),
        OpaqueFunction(function=launch_setup)
    ])