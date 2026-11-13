import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    GroupAction,
    TimerAction,
    ExecuteProcess,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Пути к пакетам
    pkg_asump_nav = get_package_share_directory('asump_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Конфигурационные файлы
    nav2_params_file = os.path.join(pkg_asump_nav, 'config', 'nav2_params.yaml')
    
    # Аргументы
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Аргументы
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # # 1. RTAB-Map в режиме локализации
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(pkg_asump_localization, 'launch', 'rtabmap_slam.launch.py')
        #     ]),
        #     launch_arguments={
        #         'use_sim_time': use_sim_time,
        #         'localization': 'true'
        #     }.items()
        # ),
        
        # # 2. Particle Filter Initializer для поиска начальной позы
        # Node(
        #     package='asump_localization',
        #     executable='pose_initializer',
        #     name='particle_filter_initializer',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'map_topic': '/map',
        #         'scan_topic': '/scan',
        #         'num_particles': 3000,
        #     }]
        # ),
        
        # # 3. Автоматический вызов сервиса инициализации
        # TimerAction(
        #     period=5.0,
        #     condition=IfCondition(auto_init_pose),
        #     actions=[
        #         LogInfo(msg='Calling /initialize_pose service...'),
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
        # ),
        
        # 4. Nav2 Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': nav2_params_file,
                'map_subscribe_transient_local': 'true',
                'enable_docking': 'false'
            }.items()
        ),

        Node(
            package='asump_nav',
            executable='mission_planner_node',
            name='mission_planner',
            output='screen',
        ),

        Node(
            package='asump_nav',
            executable='mission_executor_node',
            name='mission_executor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'action_name': 'navigate_to_pose',
                'start_service_name': '/mission/start',
                'waypoint_service_name': '/mission/get_waypoint',
                'auto_start': False,
                'retry_limit': 2,
                'skip_unreachable': True,
            }]
        ),
        
        # # 5. RViz с конфигурацией для навигации
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        #     parameters=[{'use_sim_time': use_sim_time}]
        # )
    ])