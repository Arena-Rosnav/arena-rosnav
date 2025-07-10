import os

import launch
from ament_index_python.packages import get_package_share_directory

from arena_bringup.substitutions import LaunchArgument, SelectAction


def generate_launch_description():

    ld = []

    LaunchArgument.auto_append(ld)

    namespace = LaunchArgument(
        name='namespace',
    )

    launch_human_simulator = SelectAction(launch.substitutions.LaunchConfiguration('simulator'))

    launch_human_simulator.add(
        'dummy',
        launch.actions.GroupAction([])
    )

    launch_human_simulator.add(
        'isaac',
        launch.actions.GroupAction([])
    )

    launch_human_simulator.add(
        'hunav',
        launch.actions.IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory('arena_bringup'),
                'launch/simulator/human/hunav/hunav.launch.py'
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'world_file': '',
                **namespace.dict
            }.items(),
        )
    )

    simulator = LaunchArgument(
        name='simulator',
        choices=launch_human_simulator.keys,
    )

    ld = launch.LaunchDescription([
        *ld,
        launch_human_simulator,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
