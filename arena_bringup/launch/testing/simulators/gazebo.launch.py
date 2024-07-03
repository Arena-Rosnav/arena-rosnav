import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='world',
            default_value='turtlebot3_house'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value='burger'
        ),
        launch.actions.DeclareLaunchArgument(
            name='headless',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='show_rviz',
            default_value='true',
            description='Wether to show rviz or not'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rviz_file',
            default_value='nav_LP'
        ),
        launch.actions.DeclareLaunchArgument(
            name='visualization',
            default_value=''
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'gazebo_ros'), 'launch/empty_world.launch.py')
            ),
            launch_arguments={
                'world_name': launch.substitutions.LaunchConfiguration('world'),
                'paused': 'false',
                'use_sim_time': 'true',
                'debug': 'false',
                'verbose': 'true',
                'gui': "$(eval not arg('headless') > 0)",
                'headless': "$(eval arg('headless') > 0)"
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/utils/rviz.launch.py')
            ),
            launch_arguments={
                'show_rviz': launch.substitutions.LaunchConfiguration('show_rviz')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
