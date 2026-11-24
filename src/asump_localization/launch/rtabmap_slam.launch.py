import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'
    
    asump_localization_dir = get_package_share_directory('asump_localization')
    map_tools_dir = get_package_share_directory('map_tools')
    
    rviz_config_file = os.path.join(asump_localization_dir, 'config', 'demo_robot_mapping.rviz')

    rtabmap_params = {
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
        'Vis/MinInliers': '20',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '2.5',
        'Reg/Force3DoF': 'true',
        'Grid/Sensor': '2',
        'Mem/STMSize': '30',
        'RGBD/LocalRadius': '2',
        'Icp/CorrespondenceRatio': '0.3',
        'Icp/Strategy': '0',
        'Icp/PointToPlane': 'false',
        'Icp/MaxCorrespondenceDistance': '0.15',
        'Icp/VoxelSize': '0.05',
        'Grid/RangeMax': '4.0',
        'Grid/MinClusterSize': '15',
        'Grid/RayTracing': 'true',
        'Grid/MinGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1.2',
        'cloud_voxel_size': '0.2',
        'Grid/CellSize': '0.05',
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
            parameters=[
                rtabmap_params, 
                {
                    'approx_sync_max_interval': 0.02,
                    'use_sim_time': use_sim_time
                }
            ],
            remappings=remappings
        ),

        Node(
            package='asump_localization',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'odom_frame_id': 'odom',
                    'base_frame_id': 'base_footprint'
                }
            ]
        ),
        
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[rtabmap_params],
            remappings=remappings,
            arguments=['-d']
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        OpaqueFunction(function=launch_setup)
    ])