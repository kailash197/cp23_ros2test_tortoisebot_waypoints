import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Default paths and parameters
    pkg_tortoisebot_bringup = get_package_share_directory('tortoisebot_bringup')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Include other launch files
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_tortoisebot_bringup,
                'launch',
                'bringup.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Action server node
    action_server_node = Node(
        package='tortoisebot_waypoints',
        executable='action_server',
        name='tortoisebot_as',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time_arg,
        gazebo_launch,
        action_server_node
    ])
