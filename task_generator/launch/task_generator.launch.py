import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='simulator',
            description='[dummy, gazebo]'
        ),
        launch_ros.actions.SetParameter(
            name='simulator',
            value=launch_ros.parameter_descriptions.ParameterValue(
                launch.substitutions.LaunchConfiguration('simulator'),
                value_type=str
            )
        ),

        launch.actions.DeclareLaunchArgument(
            name='entity_manager',
            description='[dummy]'
        ),
        launch_ros.actions.SetParameter(
            name='entity_manager',
            value=launch_ros.parameter_descriptions.ParameterValue(
                launch.substitutions.LaunchConfiguration('entity_manager'),
                value_type=str
            )
        ),

        launch.actions.DeclareLaunchArgument(
            name='robot',
            description='robot type [burger, jackal, ridgeback, agvota, rto, ...]'
        ),
        launch_ros.actions.SetParameter(
            name='robot',
            value=launch_ros.parameter_descriptions.ParameterValue(
                launch.substitutions.LaunchConfiguration('robot'),
                value_type=str
            )
        ),

        launch.actions.DeclareLaunchArgument(
            name='tm_robots',
        ),
        launch_ros.actions.SetParameter(
            name='tm_robots',
            value=launch_ros.parameter_descriptions.ParameterValue(
                launch.substitutions.LaunchConfiguration('tm_robots'),
                value_type=str
            )
        ),

        launch.actions.DeclareLaunchArgument(
            name='tm_obstacles',
        ),
        launch_ros.actions.SetParameter(
            name='tm_obstacles',
            value=launch_ros.parameter_descriptions.ParameterValue(
                launch.substitutions.LaunchConfiguration('tm_obstacles'),
                value_type=str
            )
        ),

        launch.actions.DeclareLaunchArgument(
            name='tm_modules',
        ),
        launch_ros.actions.SetParameter(
            name='tm_modules',
            value=launch_ros.parameter_descriptions.ParameterValue(
                launch.substitutions.LaunchConfiguration('tm_modules'),
                value_type=str
            )
        ),

        launch.actions.DeclareLaunchArgument(
            name='world',
            description='world to load'
        ),
        launch_ros.actions.SetParameter(
            name='world',
            value=launch_ros.parameter_descriptions.ParameterValue(
                launch.substitutions.LaunchConfiguration('world'),
                value_type=str
            )
        ),

        launch_ros.actions.Node(
            package='task_generator',
            executable='task_generator_node',
            name='task_generator_node',
            output='screen'
        )
        # launch_ros.actions.Node(
        #     package='task_generator',
        #     executable='server',
        #     name='task_generator_server',
        #     output='screen'
        # ),
        # launch_ros.actions.Node(
        #     package='task_generator',
        #     executable='filewatcher',
        #     name='task_generator_filewatcher',
        #     output='screen'
        # )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
