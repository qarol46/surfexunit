import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
import lifecycle_msgs.msg


def generate_launch_description():
    asump_localization_dir = get_package_share_directory('asump_localization')
    map_yaml_file = LaunchConfiguration('map_yaml_file')

    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    slam_config = os.path.join(asump_localization_dir, 'config', 'slam_localization_params.yaml')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value='/home/kirill/ros2_ws/src/surfexunit_ws/src/asump_localization/maps/new_warehouse_map.yaml',
        description='Full path to map yaml file'
    )

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

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'true',
        }.items()
    )

    # ==================== 4. Map Tools ====================
    map_tools_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            get_package_share_directory('map_tools'),
            'launch', 'map_tools.launch.py'
        ]),
        launch_arguments={'map_yaml': map_yaml_file}.items(),
    )


    lidar_odometry_node = Node(
        package='asump_localization',
        executable='lidar_odometry_node',
        name='lidar_odometry_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    auto_init_service = TimerAction(
        period=5.0,  # секунд после старта launch
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

    return LaunchDescription([
        declare_map_yaml_cmd,
        map_tools_launch,
        lidar_odometry_node,
        scan_matcher_node,
        slam_toolbox_launch,

        auto_init_service,  # <- здесь запускается автоввызов
    ])