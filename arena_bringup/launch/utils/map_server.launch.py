import os
import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import (GroupAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import PushRosNamespace, SetRemap, Node
from launch_ros.substitutions import FindPackageShare


from arena_bringup.substitutions import LaunchArgument, YAMLFileSubstitution, YAMLReplaceSubstitution, YAMLMergeSubstitution, YAMLRetrieveSubstitution


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'node_names': ['map_server', 'amcl'],
                'autostart': True,
                'use_sim_time': True,
            }]
        ),
        
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'topic_name': 'map',
                'frame_id': 'map',
                'yaml_filename': '',
            }],
        )
    ])
    
    return ld


if __name__ == '__main__':
    generate_launch_description()
