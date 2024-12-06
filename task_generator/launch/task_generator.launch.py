
import launch
import launch_ros.actions

import os
from ament_index_python.packages import get_package_share_directory


class LaunchArgument(launch.actions.DeclareLaunchArgument):
    @property
    def substitution(self):
        return launch.substitutions.LaunchConfiguration(self.name)

    @property
    def parameter(self):
        return {self.name: self.substitution}


def generate_launch_description():

    simulator = LaunchArgument(
        name='simulator',
        description='[dummy, gazebo]'
    )
    entity_manager = LaunchArgument(
        name='entity_manager',
        description='[dummy]'
    )
    robot = LaunchArgument(
        name='robot',
        description='robot type [burger, jackal, ridgeback, agvota, rto, ...]'
    )

    tm_robots = LaunchArgument(
        name='tm_robots',
    )
    tm_obstacles = LaunchArgument(
        name='tm_obstacles',
    )
    tm_modules = LaunchArgument(
        name='tm_modules',
    )
    world = LaunchArgument(
        name='world',
    )

    parameter_file = LaunchArgument(
        name='parameter_file'
    )

    task_generator_node = launch_ros.actions.Node(
        package='task_generator',
        executable='task_generator_node',
        name='task_generator_node',
        output='screen',
        parameters=[
            {
                **simulator.parameter,
                **entity_manager.parameter,
                **robot.parameter,
                **tm_robots.parameter,
                **tm_obstacles.parameter,
                **tm_modules.parameter,
                **world.parameter,
            },
            os.path.join(
                get_package_share_directory('arena_bringup'),
                'configs',
                'task_generator.yaml'),
        ],
    )

    ld = launch.LaunchDescription([
        simulator,
        entity_manager,
        robot,
        tm_robots,
        tm_obstacles,
        tm_modules,
        world,
        task_generator_node,
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
