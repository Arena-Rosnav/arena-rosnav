import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='show_rviz',
            default_value='true',
            description='Wether to show rviz or not'
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
