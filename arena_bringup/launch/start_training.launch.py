import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value='burger',
            description='model type [burger, jackal, ridgeback, agvota, rto, rto_real]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ns_prefix',
            default_value='sim'
        ),
        launch.actions.DeclareLaunchArgument(
            name='num_envs',
            default_value='1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='train_mode',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_folder_name',
            default_value='map_empty'
        ),
        launch.actions.DeclareLaunchArgument(
            name='entity_manager',
            default_value='crowdsim',
            description='[flatland, pedsim, crowdsim]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sfm',
            default_value='',
            description='sfm for crowdsim [passthrough, pysocial]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_path',
            default_value=launch.substitutions.LaunchConfiguration(
                'map_folder_name')
        ),
        launch_ros.actions.Node(
            package='arena_local_planner_drl',
            executable='action_publisher.py',
            name='action_publisher',
            parameters=[
                {
                    'SIMULATOR': 'flatland'
                },
                {
                    'local_planner': 'rosnav'
                },
                {
                    'single_env': 'false'
                },
                {
                    'map_file': launch.substitutions.LaunchConfiguration('map_folder_name')
                },
                {
                    'map_layer_path': launch.substitutions.LaunchConfiguration('map_folder_name')
                },
                {
                    'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager')
                },
                {
                    'use_sim_time': 'true'
                },
                {
                    'train_mode': launch.substitutions.LaunchConfiguration('train_mode')
                },
                {
                    'num_envs': launch.substitutions.LaunchConfiguration('num_envs')
                },
                {
                    'world_path': launch.substitutions.LaunchConfiguration('map_folder_name')
                },
                {
                    'map_path': launch.substitutions.LaunchConfiguration('map_path')
                },
                {
                    'model': launch.substitutions.LaunchConfiguration('model')
                }
            ],
            condition=launch.conditions.IfCondition(
                "$(eval arg('train_mode') == false)")
        ),
        launch_ros.actions.Node(
            package='task_generator',
            executable='server.py',
            name='task_generator_server',
            output='screen',
            parameters=[
                {
                    'SIMULATOR': 'flatland'
                },
                {
                    'local_planner': 'rosnav'
                },
                {
                    'single_env': 'false'
                },
                {
                    'map_file': launch.substitutions.LaunchConfiguration('map_folder_name')
                },
                {
                    'map_layer_path': launch.substitutions.LaunchConfiguration('map_folder_name')
                },
                {
                    'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager')
                },
                {
                    'use_sim_time': 'true'
                },
                {
                    'train_mode': launch.substitutions.LaunchConfiguration('train_mode')
                },
                {
                    'num_envs': launch.substitutions.LaunchConfiguration('num_envs')
                },
                {
                    'world_path': launch.substitutions.LaunchConfiguration('map_folder_name')
                },
                {
                    'map_path': launch.substitutions.LaunchConfiguration('map_path')
                },
                {
                    'model': launch.substitutions.LaunchConfiguration('model')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='task_generator',
            executable='filewatcher.py',
            name='task_generator_filewatcher',
            output='screen',
            parameters=[
                {
                    'SIMULATOR': 'flatland'
                },
                {
                    'local_planner': 'rosnav'
                },
                {
                    'single_env': 'false'
                },
                {
                    'map_file': launch.substitutions.LaunchConfiguration('map_folder_name')
                },
                {
                    'map_layer_path': launch.substitutions.LaunchConfiguration('map_folder_name')
                },
                {
                    'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager')
                },
                {
                    'use_sim_time': 'true'
                },
                {
                    'train_mode': launch.substitutions.LaunchConfiguration('train_mode')
                },
                {
                    'num_envs': launch.substitutions.LaunchConfiguration('num_envs')
                },
                {
                    'world_path': launch.substitutions.LaunchConfiguration('map_folder_name')
                },
                {
                    'map_path': launch.substitutions.LaunchConfiguration('map_path')
                },
                {
                    'model': launch.substitutions.LaunchConfiguration('model')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/utils/map_server.launch.py')
            ),
            launch_arguments={
                'map_file': launch.substitutions.LaunchConfiguration('map_folder_name'),
                'map_path': launch.substitutions.LaunchConfiguration('map_path')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/training/single_env_training.launch.py')
            ),
            launch_arguments={
                'ns': 'eval_sim',
                'model': launch.substitutions.LaunchConfiguration('model'),
                'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager'),
                'sfm': launch.substitutions.LaunchConfiguration('sfm'),
                'world_file': launch.substitutions.LaunchConfiguration('world_file')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/training/start_envs.launch.py')
            ),
            launch_arguments={
                'num_envs': launch.substitutions.LaunchConfiguration('num_envs'),
                'ns_prefix': launch.substitutions.LaunchConfiguration('ns_prefix'),
                'model': launch.substitutions.LaunchConfiguration('model'),
                'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager'),
                'sfm': launch.substitutions.LaunchConfiguration('sfm')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
