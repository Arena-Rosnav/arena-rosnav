import os
import launch
import launch.actions
import launch.substitutions
from ament_index_python.packages import get_package_share_directory

from arena_bringup.substitutions import LaunchArgument, SelectAction


def generate_launch_description():

    simulator = LaunchArgument(
        name='simulator',
        choices=['dummy', 'gazebo', 'isaac'],
    )

    use_sim_time = LaunchArgument(
        name='use_sim_time',
    )

    headless = LaunchArgument(
        name='headless',
        default_value='False',
    )

    # TODO temporary
    world = LaunchArgument(
        name='world'
    )

    launch_simulator = SelectAction(simulator.substitution)

    launch_simulator.add(
        'gazebo',
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/shared/simulator/gazebo/gazebo.launch.py')
            ),
            launch_arguments={
                **use_sim_time.dict,
                **headless.dict,
                **world.dict,
            }.items(),
        )
    )

    launch_simulator.add(
        'isaac',
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'arena_bringup'), 'launch/shared/simulator/isaac/isaac.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time.substitution,
                # 'headless': headless.substitution
            }.items(),
        )
    )

    ld = launch.LaunchDescription([
        simulator,
        headless,
        world,
        launch_simulator,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
