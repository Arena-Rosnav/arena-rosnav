import launch
import os
from ament_index_python.packages import get_package_share_directory

from arena_bringup.substitutions import LaunchArgument, SelectAction


def generate_launch_description():
    # Current path to the workspace
    entity_manager = LaunchArgument(
        name='entity_manager',
        choices=['dummy', 'hunav','isaac'],
    )

    launch_entity_manager = SelectAction(entity_manager.substitution)

    launch_entity_manager.add(
        'hunav',
        launch.actions.IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory('arena_bringup'),
                'launch/shared/entity_manager/hunav/hunav.launch.py'
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'world_file': ''
            }.items(),
        )
    )

    ld = launch.LaunchDescription([
        entity_manager,
        launch_entity_manager,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
