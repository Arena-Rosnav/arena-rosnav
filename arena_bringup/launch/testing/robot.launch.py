import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='inter_planner'
        ),
        launch.actions.DeclareLaunchArgument(
            name='local_planner',
            default_value='',
            description='local planner type [teb, dwa, mpc, rlca, arena, rosnav]'
        ),
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
            name='frame',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='SIMULATOR',
            default_value='flatland'
        ),
        launch.actions.DeclareLaunchArgument(
            name='train_mode',
            default_value='false',
            description='If false, start the Rosnav Deployment Nodes'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value=launch.substitutions.LaunchConfiguration('name'),
            description='DRL agent name to be deployed'
        ),
        launch.actions.DeclareLaunchArgument(
            name='complexity',
            default_value='1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='record_data',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='record_data_dir',
            default_value='auto:'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
