import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='num_envs',
            default_value='1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ns_prefix',
            default_value='sim'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model'
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
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/training/single_env_training.launch.py')
            ),
            launch_arguments={
                'ns': launch.substitutions.LaunchConfiguration('ns_prefix'),
                'model': launch.substitutions.LaunchConfiguration('model')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/training/start_envs.launch.py')
            ),
            launch_arguments={
                'num_envs': "$(eval arg('num_envs') - 1)",
                'ns_prefix': launch.substitutions.LaunchConfiguration('ns_prefix'),
                'model': launch.substitutions.LaunchConfiguration('model')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
