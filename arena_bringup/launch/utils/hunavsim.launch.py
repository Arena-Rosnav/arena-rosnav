import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Agent Manager Node
        Node(
            package='hunav_agent_manager',  
            executable='hunav_agent_manager',  
            name='hunav_agent_manager',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        )
    ])