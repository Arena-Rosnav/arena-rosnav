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
            package='arena_timespace_planner',
            executable='time_space_planner_node',
            name='time_space',
            output='screen',
            parameters=[
                get_package_share_directory(
                    'arena_bringup') + '/launch/plan_fsm_param.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
