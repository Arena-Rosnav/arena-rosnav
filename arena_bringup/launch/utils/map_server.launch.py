import os
import sys

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='map_file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_path'
        ),
        launch_ros.actions.Node(
            package='map_server',
            executable='map_server',
            name='map_server',
            condition=UnlessCondition(LaunchConfiguration('map_file').perform == 'dynamic_map')
        ),
        launch_ros.actions.Node(
            package='map_generator',
            executable='map_generator_node.py',
            name='map_generator_node',
            condition=IfCondition(LaunchConfiguration('map_file').perform == 'dynamic_map')
        ),
        launch_ros.actions.Node(
            package='map_generator',
            executable='map_server.py',
            name='map_server_starter'
        ),
        launch_ros.actions.Node(
            package='map_distance_server',
            executable='map_distance_node.py',
            name='distance_server',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='map_clock_simulator',
            executable='node.py',
            name='map_clock_simulator',
            output='screen'
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()

