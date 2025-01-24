
import os
import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        # launch.actions.IncludeLaunchDescription(
        #     launch.launch_description_sources.PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('nav2_bringup'), 'launch/bringup_launch.py')),
        #     launch_arguments={
        #         'autostart': 'False',
        #         'use_composition': 'False',
        #         'use_namespace': 'True',
        #         'namespace': 'nav2'
        #     }.items(),
        # ),
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[
                {
                    'node_names': ['map_server', 'amcl'],
                    'autostart': True,
                    'use_sim_time': True,
                }
            ]
        ),
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'topic_name': 'map',
                'frame_id': 'map',
                'yaml_filename': '',
            }],
        ),
        launch_ros.actions.Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
