import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition, IfCondition
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_rviz = LaunchConfiguration('use_rviz')

    # Получаем путь к пакету asump_description
    pkg_path = get_package_share_directory('asump_description')
    
    # Путь к файлу описания робота
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    rviz_config = os.path.join(pkg_path, 'config', 'rsp.rviz')

    # Генерируем описание робота из xacro
    robot_description_config = Command([
        'xacro ', xacro_file, 
        ' use_ros2_control:=', use_ros2_control,
        ' use_sim_time:=', use_sim_time
    ])
    
    robot_description = ParameterValue(robot_description_config, value_type=str)
    params = {
        'robot_description': robot_description, 
        'use_sim_time': use_sim_time
    }
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_ros2_control)
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=UnlessCondition(use_ros2_control)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
 
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Использовать симулированное время если true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Использовать ros2_control если true'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Использовать rviz если true'),

        node_robot_state_publisher,
        node_joint_state_publisher,
        node_joint_state_publisher_gui,
        rviz_node
    ])