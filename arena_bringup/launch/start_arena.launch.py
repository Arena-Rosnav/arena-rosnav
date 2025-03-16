import os

import launch
from ament_index_python.packages import get_package_share_directory

import launch_ros.actions


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
            name='robot',
            default_value='jackal',
            description='robot model type [burger, jackal, ridgeback, agvota, rto, ...]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='inter_planner',
            default_value='navigate_w_replanning_time',
            description='inter planner type (Behavior Tree)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='local_planner',
            default_value='dwb',
            description='local planner type [teb, dwa, mpc, rlca, arena, rosnav, cohan]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='global_planner',
            default_value='navfn',
            description='global planner type [navfn]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='simulator',
            default_value='dummy',
        ),
        launch.actions.DeclareLaunchArgument(
            name='headless',
            default_value='0',
            description='0 = show all, 1 = show only rviz, 2 = show nothing'
        ),
        launch.actions.DeclareLaunchArgument(
            name='entity_manager',
            default_value='dummy',
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
            default_value=launch.substitutions.LaunchConfiguration('robot'),
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
            default_value='rviz_ui'  # TODO breaks launch if empty
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
            name='world',
            default_value='map_empty',
            description='world to load'
        ),
        launch.actions.DeclareLaunchArgument(
            name='global_frame_id',
            default_value='map'
        ),
        launch.actions.DeclareLaunchArgument(
            name='odom_frame_id',
            default_value='odom'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='random_spawn_test',
            default_value='false',
            description='test parameter for the random spawning of entities'
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'task_generator'), 'launch/task_generator.launch.py')
            ),
            launch_arguments={
                'simulator': launch.substitutions.LaunchConfiguration('simulator'),
                'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager'),
                'tm_obstacles': launch.substitutions.LaunchConfiguration('tm_obstacles'),
                'tm_robots': launch.substitutions.LaunchConfiguration('tm_robots'),
                'tm_modules': launch.substitutions.LaunchConfiguration('tm_modules'),
                'robot': launch.substitutions.LaunchConfiguration('robot'),
                'inter_planner': launch.substitutions.LaunchConfiguration('inter_planner'),
                'local_planner': launch.substitutions.LaunchConfiguration('local_planner'),
                'global_planner': launch.substitutions.LaunchConfiguration('global_planner'),
                'world': launch.substitutions.LaunchConfiguration('world'),
                'parameter_file': os.path.join(get_package_share_directory('arena_bringup'), 'configs', 'task_generator.yaml'),
            }.items(),
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/utils/map_server.launch.py')
            )
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/shared/simulator/simulator.launch.py')
            ),
            launch_arguments={
                'simulator': launch.substitutions.LaunchConfiguration('simulator'),
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'headless': launch.substitutions.LaunchConfiguration('headless'),
                'world': launch.substitutions.LaunchConfiguration('world'),
            }.items(),
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/shared/entity_manager/entity_manager.launch.py')
            ),
            launch_arguments={
                'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager'),
                'world_file': launch.substitutions.LaunchConfiguration('world'),
            }.items()
        ),

        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', '/path/to/default.rviz']
        ),

        # Add robot model visualizer
        launch_ros.actions.Node(
            package='rviz_utils',
            executable='visualize_robot_model',
            name='visualize_robot_model',
            parameters=[
                {'robot_names': [launch.substitutions.LaunchConfiguration('robot')]},
                {'complexity': launch.substitutions.LaunchConfiguration('complexity')}
            ],
            output='screen'
        ),

        # Add RViz config generator for creating robot groups with visualizations
        launch_ros.actions.Node(
            package='rviz_utils',
            executable='rviz_config',
            name='rviz_config_generator',
            parameters=[
                {'robot_names': [launch.substitutions.LaunchConfiguration('robot')]}
            ],
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()


# launch_ros.actions.Node(
    #     package='rviz_utils',
    #     executable='create_config_file.py',
    #     name='rviz_config_file_creator',
    #     parameters=[
    #         {
    #             'debug_mode': 'true'
    #         },
    #         {
    #             'desired_resets': launch.substitutions.LaunchConfiguration('desired_resets')
    #         },
    #         {
    #             'task_generator_server/timeout': launch.substitutions.LaunchConfiguration('timeout')
    #         },
    #         {
    #             'headless': launch.substitutions.LaunchConfiguration('headless')
    #         },
    #         {
    #             'record_data_dir': launch.substitutions.LaunchConfiguration('record_data_dir')
    #         },
    #         {
    #             'robot_setup_file': launch.substitutions.LaunchConfiguration('robot_setup_file')
    #         },
    #         {
    #             'model': launch.substitutions.LaunchConfiguration('model')
    #         },
    #         {
    #             'inter_planner': launch.substitutions.LaunchConfiguration('inter_planner')
    #         },
    #         {
    #             'local_planner': launch.substitutions.LaunchConfiguration('local_planner')
    #         },
    #         {
    #             'agent_name': launch.substitutions.LaunchConfiguration('agent_name')
    #         },
    #         {
    #             'complexity': launch.substitutions.LaunchConfiguration('complexity')
    #         },
    #         {
    #             'reset_remove_all': launch.substitutions.LaunchConfiguration('reset_remove_all')
    #         },
    #         {
    #             'use_sim_time': 'true'
    #         },
    #         {
    #             'rosnav_move_base': 'true'
    #         },
    #         {
    #             'model': launch.substitutions.LaunchConfiguration('model')
    #         },
    #         {
    #             'simulator': launch.substitutions.LaunchConfiguration('simulator')
    #         },
    #         {
    #             'tm_robots': launch.substitutions.LaunchConfiguration('tm_robots')
    #         },
    #         {
    #             'tm_obstacles': launch.substitutions.LaunchConfiguration('tm_obstacles')
    #         },
    #         {
    #             'tm_modules': launch.substitutions.LaunchConfiguration('tm_modules')
    #         },
    #         {
    #             '/benchmark_resume': launch.substitutions.LaunchConfiguration('benchmark_resume')
    #         },
    #         {
    #             'map_path': launch.substitutions.LaunchConfiguration('map_path')
    #         },
    #         {
    #             'train_mode': 'false'
    #         },
    #         {
    #             'show_viz': launch.substitutions.LaunchConfiguration('show_rviz')
    #         },
    #         {
    #             'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager')
    #         },
    #         {
    #             'world_path': launch.substitutions.LaunchConfiguration('map_file')
    #         },
    #         {
    #             'map_layer_path': launch.substitutions.LaunchConfiguration('map_file')
    #         },
    #         {
    #             'map_file': launch.substitutions.LaunchConfiguration('map_file')
    #         },
    #         {
    #             'robot_name': launch.substitutions.LaunchConfiguration('model')
    #         }
    #     ]
    # ),
    # launch_ros.actions.Node(
    #     package='rviz_utils',
    #     executable='visualize_robot_model.py',
    #     name='visualize_robot_model',
    #     output='screen',
    #     parameters=[
    #         {
    #             'debug_mode': 'true'
    #         },
    #         {
    #             'desired_resets': launch.substitutions.LaunchConfiguration('desired_resets')
    #         },
    #         {
    #             'task_generator_server/timeout': launch.substitutions.LaunchConfiguration('timeout')
    #         },
    #         {
    #             'headless': launch.substitutions.LaunchConfiguration('headless')
    #         },
    #         {
    #             'record_data_dir': launch.substitutions.LaunchConfiguration('record_data_dir')
    #         },
    #         {
    #             'robot_setup_file': launch.substitutions.LaunchConfiguration('robot_setup_file')
    #         },
    #         {
    #             'model': launch.substitutions.LaunchConfiguration('model')
    #         },
    #         {
    #             'inter_planner': launch.substitutions.LaunchConfiguration('inter_planner')
    #         },
    #         {
    #             'local_planner': launch.substitutions.LaunchConfiguration('local_planner')
    #         },
    #         {
    #             'agent_name': launch.substitutions.LaunchConfiguration('agent_name')
    #         },
    #         {
    #             'complexity': launch.substitutions.LaunchConfiguration('complexity')
    #         },
    #         {
    #             'reset_remove_all': launch.substitutions.LaunchConfiguration('reset_remove_all')
    #         },
    #         {
    #             'use_sim_time': 'true'
    #         },
    #         {
    #             'rosnav_move_base': 'true'
    #         },
    #         {
    #             'model': launch.substitutions.LaunchConfiguration('model')
    #         },
    #         {
    #             'simulator': launch.substitutions.LaunchConfiguration('simulator')
    #         },
    #         {
    #             'tm_robots': launch.substitutions.LaunchConfiguration('tm_robots')
    #         },
    #         {
    #             'tm_obstacles': launch.substitutions.LaunchConfiguration('tm_obstacles')
    #         },
    #         {
    #             'tm_modules': launch.substitutions.LaunchConfiguration('tm_modules')
    #         },
    #         {
    #             '/benchmark_resume': launch.substitutions.LaunchConfiguration('benchmark_resume')
    #         },
    #         {
    #             'map_path': launch.substitutions.LaunchConfiguration('map_path')
    #         },
    #         {
    #             'train_mode': 'false'
    #         },
    #         {
    #             'show_viz': launch.substitutions.LaunchConfiguration('show_rviz')
    #         },
    #         {
    #             'entity_manager': launch.substitutions.LaunchConfiguration('entity_manager')
    #         },
    #         {
    #             'world_path': launch.substitutions.LaunchConfiguration('map_file')
    #         },
    #         {
    #             'map_layer_path': launch.substitutions.LaunchConfiguration('map_file')
    #         },
    #         {
    #             'map_file': launch.substitutions.LaunchConfiguration('map_file')
    #         },
    #         {
    #             'robot_name': launch.substitutions.LaunchConfiguration('model')
    #         }
    #     ]
    # ),
    # launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory(
    #             'arena_bringup'), 'launch/testing/simulators/flatland.launch.py')
    #     ),
    #     launch_arguments={
    #         'visualization': launch.substitutions.LaunchConfiguration('visualization'),
    #         'rviz_file': launch.substitutions.LaunchConfiguration('rviz_file'),
    #         'model': launch.substitutions.LaunchConfiguration('model'),
    #         'show_rviz': launch.substitutions.LaunchConfiguration('show_rviz'),
    #         'headless': launch.substitutions.LaunchConfiguration('headless')
    #     }.items()
    # ),
