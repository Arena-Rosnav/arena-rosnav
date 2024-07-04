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

