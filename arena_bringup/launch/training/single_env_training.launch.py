import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='ns'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value='jackal'
        ),
        launch.actions.DeclareLaunchArgument(
            name='entity_manager',
            default_value='crowdsim'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='sfm',
            default_value=''
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
