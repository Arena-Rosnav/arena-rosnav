import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='desired_resets',
            default_value='10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value='burger',
            description='robot model type [burger, jackal, ridgeback, agvota, rto, ...]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='inter_planner',
            default_value='bypass',
            description='inter planner type [bypass, shortsighted, polite, aggressive, sideways]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='local_planner',
            default_value='teb',
            description='local planner type [teb, dwa, mpc, rlca, arena, rosnav, cohan]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='simulator',
            default_value='flatland',
            description='[flatland, gazebo]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='headless',
            default_value='0',
            description='0 = show all, 1 = show only rviz, 2 = show nothing'
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
            name='complexity',
            default_value='1',
            description='1 = Map known, Position known; 2 = Map known, Position unknown (AMCL); 3 = Map unknown, Position unknown (SLAM)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value=launch.substitutions.LaunchConfiguration('model'),
            description='DRL agent name to be deployed'
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_setup_file',
            default_value='',
            description=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='record_data',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='record_data_dir',
            default_value='auto:'
        ),
        launch.actions.DeclareLaunchArgument(
            name='tm_robots',
            default_value='random'
        ),
        launch.actions.DeclareLaunchArgument(
            name='tm_obstacles',
            default_value='random'
        ),
        launch.actions.DeclareLaunchArgument(
            name='tm_modules',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='benchmark_resume',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='visualization',
            default_value='rviz',
            description='[rviz, flatland]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='show_rviz',
            default_value='true',
            description='Enables rviz in gazebo'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rviz_file',
            default_value='nav'
        ),
        launch.actions.DeclareLaunchArgument(
            name='auto_reset',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='reset_remove_all',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_file',
            default_value='map_empty',
            description='[map1, floor, indoor, indoor_map1, map_small, map_empty]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_path',
            default_value=launch.substitutions.LaunchConfiguration('map_file')
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_file',
            default_value=launch.substitutions.LaunchConfiguration('map_file'),
            description='set to generated_world to replace occupancy map with obstacles and walls (NOT IMPLEMENTED YET)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='global_frame_id',
            default_value='map'
        ),
        launch.actions.DeclareLaunchArgument(
            name='odom_frame_id',
            default_value='odom'
        ),
        launch_ros.actions.Node(
            package='rviz_utils',
            executable='create_config_file.py',
            name='rviz_config_file_creator',
            parameters=[
                {
                    'debug_mode': 'true'
                },
                {
                    'desired_resets': launch.substitutions.LaunchConfiguration('desired_resets')
                },
                {
                    'task_generator_server/timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'headless': launch.substitutions.LaunchConfiguration('headless')
                },
                {
                    'record_data_dir': launch.substitutions.LaunchConfiguration('record_data_dir')
                },
                {
                    'robot_setup_file': launch.substitutions.LaunchConfiguration('robot_setup_file')
                },
                {
                    'model': launch.substitutions.LaunchConfiguration('model')
                },
                {
                    'inter_planner': launch.substitutions.LaunchConfiguration('inter_planner')
                },
                {
                    'local_planner': launch.substitutions.LaunchConfiguration('local_planner')
                },
                {
                    'agent_name': launch.substitutions.LaunchConfiguration('agent_name')
                },
                {
                    'complexity': launch.substitutions.LaunchConfiguration('complexity')
                },
                {
                    'reset_remove_all': launch.substitutions.LaunchConfiguration('reset_remove_all')
                },
                {
                    'use_sim_time': 'true'
                },
                {
                    'rosnav_move_base': 'true'
                },
                {
                    'model': launch.substitutions.LaunchConfiguration('model')
                },
                {
                    'simulator': launch.substitutions.LaunchConfiguration('simulator')
                },
                {
                    'tm_robots': launch.substitutions.LaunchConfiguration('tm_robots')
                },
                {
                    'tm_obstacles': launch.substitutions.LaunchConfiguration('tm_obstacles')
                },
                {
                    'tm_modules': launch.substitutions.LaunchConfiguration('tm_modules')
                },
                {
                    '/benchmark_resume': launch.substitutions.LaunchConfiguration('benchmark_resume')
                },
                {
                    'map_path': launch.substitutions.LaunchConfiguration('map_path')
                },
                {
                    'train_mode': 'false'
                },
                {
                    'show_viz': launch.substitutions.LaunchConfiguration('show_rviz')
                },
                {
                    'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager')
                },
                {
                    'world_path': launch.substitutions.LaunchConfiguration('map_file')
                },
                {
                    'map_layer_path': launch.substitutions.LaunchConfiguration('map_file')
                },
                {
                    'map_file': launch.substitutions.LaunchConfiguration('map_file')
                },
                {
                    'robot_name': launch.substitutions.LaunchConfiguration('model')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rviz_utils',
            executable='visualize_robot_model.py',
            name='visualize_robot_model',
            output='screen',
            parameters=[
                {
                    'debug_mode': 'true'
                },
                {
                    'desired_resets': launch.substitutions.LaunchConfiguration('desired_resets')
                },
                {
                    'task_generator_server/timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'headless': launch.substitutions.LaunchConfiguration('headless')
                },
                {
                    'record_data_dir': launch.substitutions.LaunchConfiguration('record_data_dir')
                },
                {
                    'robot_setup_file': launch.substitutions.LaunchConfiguration('robot_setup_file')
                },
                {
                    'model': launch.substitutions.LaunchConfiguration('model')
                },
                {
                    'inter_planner': launch.substitutions.LaunchConfiguration('inter_planner')
                },
                {
                    'local_planner': launch.substitutions.LaunchConfiguration('local_planner')
                },
                {
                    'agent_name': launch.substitutions.LaunchConfiguration('agent_name')
                },
                {
                    'complexity': launch.substitutions.LaunchConfiguration('complexity')
                },
                {
                    'reset_remove_all': launch.substitutions.LaunchConfiguration('reset_remove_all')
                },
                {
                    'use_sim_time': 'true'
                },
                {
                    'rosnav_move_base': 'true'
                },
                {
                    'model': launch.substitutions.LaunchConfiguration('model')
                },
                {
                    'simulator': launch.substitutions.LaunchConfiguration('simulator')
                },
                {
                    'tm_robots': launch.substitutions.LaunchConfiguration('tm_robots')
                },
                {
                    'tm_obstacles': launch.substitutions.LaunchConfiguration('tm_obstacles')
                },
                {
                    'tm_modules': launch.substitutions.LaunchConfiguration('tm_modules')
                },
                {
                    '/benchmark_resume': launch.substitutions.LaunchConfiguration('benchmark_resume')
                },
                {
                    'map_path': launch.substitutions.LaunchConfiguration('map_path')
                },
                {
                    'train_mode': 'false'
                },
                {
                    'show_viz': launch.substitutions.LaunchConfiguration('show_rviz')
                },
                {
                    'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager')
                },
                {
                    'world_path': launch.substitutions.LaunchConfiguration('map_file')
                },
                {
                    'map_layer_path': launch.substitutions.LaunchConfiguration('map_file')
                },
                {
                    'map_file': launch.substitutions.LaunchConfiguration('map_file')
                },
                {
                    'robot_name': launch.substitutions.LaunchConfiguration('model')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/utils/entity_manager.launch.py')
            ),
            launch_arguments={
                'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager'),
                'world_file': launch.substitutions.LaunchConfiguration('world_file'),
                'sfm': launch.substitutions.LaunchConfiguration('sfm')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/testing/simulators/flatland.launch.py')
            ),
            launch_arguments={
                'visualization': launch.substitutions.LaunchConfiguration('visualization'),
                'rviz_file': launch.substitutions.LaunchConfiguration('rviz_file'),
                'model': launch.substitutions.LaunchConfiguration('model'),
                'show_rviz': launch.substitutions.LaunchConfiguration('show_rviz'),
                'headless': launch.substitutions.LaunchConfiguration('headless')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/testing/simulators/gazebo.launch.py')
            ),
            launch_arguments={
                'model': launch.substitutions.LaunchConfiguration('model'),
                'rviz_file': launch.substitutions.LaunchConfiguration('rviz_file'),
                'show_rviz': launch.substitutions.LaunchConfiguration('show_rviz'),
                'world': launch.substitutions.LaunchConfiguration('world_file'),
                'headless': launch.substitutions.LaunchConfiguration('headless')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/utils/map_server.launch.py')
            ),
            launch_arguments={
                'map_file': launch.substitutions.LaunchConfiguration('map_file'),
                'map_path': launch.substitutions.LaunchConfiguration('map_path')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/utils/fake_localization.launch.py')
            ),
            launch_arguments={
                'ns': '',
                'robot_name': launch.substitutions.LaunchConfiguration('model'),
                'global_frame_id': launch.substitutions.LaunchConfiguration('global_frame_id'),
                'odom_frame_id': launch.substitutions.LaunchConfiguration('odom_frame_id')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/testing/task_generator.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
