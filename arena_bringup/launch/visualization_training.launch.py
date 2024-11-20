import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='ns',
            default_value='sim_1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rviz_file',
            default_value='nav'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='flatland_rviz',
            output='screen',
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_rviz'))
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
