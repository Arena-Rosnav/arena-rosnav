import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='config',
            default_value="''"
        ),
        launch_ros.actions.Node(
            package='training',
            executable='train_agent.py',
            name='train_agent',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
