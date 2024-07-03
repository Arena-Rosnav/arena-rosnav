import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value='rule_04'
        ),
        launch_ros.actions.Node(
            package='arena_local_planner_drl',
            executable='drl_agent_node.py',
            name='DRL_local_planner'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
