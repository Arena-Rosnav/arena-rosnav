
import launch
import launch_ros.actions

import os
from ament_index_python.packages import get_package_share_directory

from arena_bringup.substitutions import LaunchArgument


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

    local_planner = LaunchArgument(
        name='local_planner',
    )
    inter_planner = LaunchArgument(
        name='inter_planner',
    )
    global_planner = LaunchArgument(
        name='global_planner',
    )
    record_data_dir = LaunchArgument(
        name='record_data_dir',
        default_value='',
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
                **simulator.str_param,
                **entity_manager.str_param,
                **robot.str_param,
                **tm_robots.str_param,
                **tm_obstacles.str_param,
                **tm_modules.str_param,
                **world.str_param,
                **inter_planner.str_param,
                **local_planner.str_param,
                **global_planner.str_param,
                **record_data_dir.str_param,
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
        record_data_dir,
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
