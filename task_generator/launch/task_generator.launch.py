
import os
import typing

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

from arena_bringup.substitutions import (CurrentNamespaceSubstitution,
                                         LaunchArgument)
from arena_bringup.future import PythonExpression


def generate_launch_description():

    bringup_dir = get_package_share_directory('arena_bringup')

    ld_items = []
    LaunchArgument.auto_append(ld_items)

    namespace = LaunchArgument(
        name='namespace',
        default_value='task_generator_node'
    )

    sim = LaunchArgument(
        name='sim',
        description='[dummy, gazebo, isaac]'
    )
    human = LaunchArgument(
        name='human',
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

    headless = LaunchArgument(
        name='headless',
        default_value='False',
    )
    reference = LaunchArgument(
        name='reference',
        default_value='[0, 0]',
    )
    prefix = LaunchArgument(
        name='prefix',
        default_value='',
    )

    map_server_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch/utils/map_server.launch.py')
        )
    )

    # Hunavsim Pedestrians in rviz
    pedestrian_marker_node = launch_ros.actions.Node(
        package="rviz_utils",
        executable="pedestrian_marker_publisher",
        name="pedestrian_marker_publisher",
        parameters=[
            {"use_sim_time": True},
            {"body_height": 1.6},
            {"body_radius": 0.25},
            {"head_radius": 0.15},
            {"arrow_length": 0.6},
            {"show_labels": True},
            {"show_velocity_arrows": True},
            {"show_orientation_arrows": True},
            {"namespace": namespace.substitution},
        ],
        output="screen",
        condition=launch.conditions.IfCondition(PythonExpression(['"', human.substitution, '" == "hunav"'])),
    )
    # Start the rviz config generator which launches also rviz2 with desired config file
    rviz_node = launch_ros.actions.Node(
        package="rviz_utils",
        executable="rviz_config",
        name="rviz_config_generator",
        arguments=[
            CurrentNamespaceSubstitution(),
        ],
        parameters=[
            {
                "use_sim_time": True,
                "origin": reference.param_value(typing.List[float]),
            }
        ],
        output="screen",
        condition=launch.conditions.UnlessCondition(headless.substitution),
    )

    task_generator_node = launch_ros.actions.Node(
        package='task_generator',
        executable='task_generator_node',
        name=namespace.substitution,
        output='screen',
        parameters=[
            {
                **sim.str_param,
                **human.str_param,
                **robot.str_param,
                **tm_robots.str_param,
                **tm_obstacles.str_param,
                **tm_modules.str_param,
                **world.str_param,
                **inter_planner.str_param,
                **local_planner.str_param,
                **global_planner.str_param,
                **record_data_dir.str_param,
                **reference.param(typing.List[float]),
                **prefix.str_param,
            },
            parameter_file.substitution,
        ],
    )

    ld = launch.LaunchDescription([
        *ld_items,
        launch.actions.GroupAction([
            launch_ros.actions.PushRosNamespace(namespace=namespace.substitution),
            map_server_node,
            pedestrian_marker_node,
            rviz_node
        ]),
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
