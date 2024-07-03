import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='ns',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_name',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='global_frame_id'
        ),
        launch.actions.DeclareLaunchArgument(
            name='odom_frame_id'
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tfpublisher'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
