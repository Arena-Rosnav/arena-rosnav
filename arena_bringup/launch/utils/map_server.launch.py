import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            parameters=[{
                'node_names': ['map_server'],
                'autostart': True,
                'use_sim_time': True,
                'bond_timeout': 0.0,
            }]
        ),

        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'topic_name': 'map',
                'frame_id': 'map',
                'yaml_filename': '',
                'use_sim_time': True,
            }],
        )
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
