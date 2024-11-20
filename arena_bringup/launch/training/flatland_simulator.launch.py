import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='ns'
        ),
        launch_ros.actions.Node(
            package='flatland_server',
            executable='flatland_server',
            name='flatland_server',
            output='screen'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/utils/rviz.launch.py')
            ),
            launch_arguments={
                'show_rviz': 'false'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
