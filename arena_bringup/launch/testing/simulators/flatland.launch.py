import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz_file',
            default_value='nav_LP'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value='model'
        ),
        launch.actions.DeclareLaunchArgument(
            name='headless',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='show_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='visualization',
            default_value='rviz',
            description='[rviz, flatland]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_pose_x',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_pose_y',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_pose_a',
            default_value='0.0'
        ),
        launch_ros.actions.Node(
            package='flatland_server',
            executable='flatland_server',
            name='flatland_server',
            parameters=[
                {
                    'synchronous': 'true'
                },
                {
                    '~debug/verbosity': '1'
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
