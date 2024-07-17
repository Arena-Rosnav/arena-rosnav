import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='task_generator',
            executable='task_generator_node',
            name='task_generator_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='task_generator',
            executable='server',
            name='task_generator_server',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='task_generator',
            executable='filewatcher',
            name='task_generator_filewatcher',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
