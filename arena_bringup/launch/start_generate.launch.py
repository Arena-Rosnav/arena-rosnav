import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='verbose',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_name',
            default_value=''
        ),
        launch_ros.actions.Node(
            package='grid_map_generator',
            executable='grid_map_generator_node.py',
            name='grid_map_generator',
            output='screen',
            parameters=[
                {
                    'world_name': launch.substitutions.LaunchConfiguration('world_name')
                }
            ],
            condition=launch.conditions.IfCondition(
                "$(eval arg('world_name') != '')")
        ),
        launch_ros.actions.Node(
            package='grid_map_generator',
            executable='grid_map_generator_node.py',
            name='grid_map_generator',
            output='screen',
            parameters=[
                {
                    'world_name': launch.substitutions.LaunchConfiguration('world_name')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'gazebo_ros'), 'launch/empty_world.launch.py')
            ),
            launch_arguments={
                'world_name': launch.substitutions.LaunchConfiguration('world_name'),
                'gui': launch.substitutions.LaunchConfiguration('gui'),
                'verbose': launch.substitutions.LaunchConfiguration('verbose')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
