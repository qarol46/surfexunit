import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('asump_localization')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_config = os.path.join(pkg_share, 'config', 'slam_mapper_params.yaml')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 automatically'
    )

    lidar_odometry_node = Node(
        package='asump_localization',
        executable='lidar_odometry_node',
        name='lidar_odometry_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rgbd_mapper_node = Node(
        package='asump_localization',
        executable='rgbd_mapper_node',
        name='rgbd_mapper_node',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'cloud_topic': '/depth_camera/depth/points',
            'map_topic': 'rgbd_map',
            'voxel_leaf_size': 0.25,
            'keyframe_translation_threshold': 0.5,
            'keyframe_rotation_threshold': 0.5,
            'default_save_path': '/home/kirill/ros2_ws/src/surfexunit_ws/src/asump_localization/maps/rgbd_map.pcd',
            'save_binary': True,
        }],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    rviz_config_file = os.path.join(pkg_share, 'config', 'mapper.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        lidar_odometry_node,
        rgbd_mapper_node,
        slam_toolbox_launch,
        rviz_node
    ])