from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'
    
    localization = LaunchConfiguration('localization').perform(context)
    
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
        
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityByTime': 'false',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Reg/Strategy': '1',
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
        'Icp/VoxelSize': '0.05'
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
        
        # # Визуализация
        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmap_viz',
        #     output='screen',
        #     parameters=[parameters],
        #     remappings=remappings
        # ),
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
            default_value='false',
            description='Launch in localization mode'
        ),
        OpaqueFunction(function=launch_setup)
    ])