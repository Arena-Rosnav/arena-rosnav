import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='entity_manager',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='sfm',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='scene_file',
            default_value=launch.substitutions.LaunchConfiguration(
                'world_file')
        ),
        launch.actions.DeclareLaunchArgument(
            name='pedsim',
            default_value="$(eval entity_manager in ('pedsim', 'crowdsim'))"
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
