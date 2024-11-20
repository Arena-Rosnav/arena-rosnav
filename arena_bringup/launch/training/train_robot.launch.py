import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='name',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='namespace',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='sim_namespace',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='inter_planner',
            default_value='bypass'
        ),
        launch.actions.DeclareLaunchArgument(
            name='local_planner',
            default_value='rosnav'
        ),
        launch.actions.DeclareLaunchArgument(
            name='train_mode',
            default_value='true',
            description='If false, start the Rosnav Deployment Nodes'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value=launch.substitutions.LaunchConfiguration('name'),
            description='DRL agent name to be deployed'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
