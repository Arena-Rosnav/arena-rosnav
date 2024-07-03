import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='train_mode',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='look_ahead_distance'
        ),
        launch.actions.DeclareLaunchArgument(
            name='tolerance_approach'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout_goal'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout_subgoal'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ns',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_name'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
