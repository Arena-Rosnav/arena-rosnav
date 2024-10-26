import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file')
    
    # Get the hunav_sim directory
    hunav_sim_dir = get_package_share_directory('hunav_sim')
    
    # Hunavsim node
    hunavsim_node = Node(
        package='hunav_sim',
        executable='hunav_sim_node',
        name='hunav_sim',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'world_file': world_file},
            # Hunavsim specific parameters
            {'max_vel': 0.3},
            {'force_factor_desired': 1.0},
            {'force_factor_obstacle': 1.0},
            {'force_factor_social': 5.0},
            {'force_factor_robot': 0.0},
        ]
    )
    
    # Agent manager node
    agent_manager_node = Node(
        package='hunav_sim',
        executable='hunav_agent_manager_node',
        name='hunav_agent_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty.world',
            description='World file for simulation'
        ),
        
        # Nodes
        hunavsim_node,
        agent_manager_node,
    ])