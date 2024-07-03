import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='ns',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='max_vel',
            default_value='3.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='max_acc',
            default_value='2.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='max_jerk',
            default_value='4.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='lambda_heu',
            default_value='0.1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='plan_horizon',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_name'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
