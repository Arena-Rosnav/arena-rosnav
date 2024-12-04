import launch
import os
from ament_index_python.packages import get_package_share_directory


class LaunchArgument(launch.actions.DeclareLaunchArgument):
    @property
    def substitution(self):
        return launch.substitutions.LaunchConfiguration(self.name)

    @property
    def parameter(self):
        return {self.name: self.substitution}


class SelectAction:
    _actions: list[launch.actions.GroupAction]
    _selector: launch.substitutions.LaunchConfiguration

    def __init__(
        self,
        selector: launch.substitutions.LaunchConfiguration
    ) -> None:
        self._actions = []
        self._selector = selector

    def add(self, value: str, action: launch.Action):
        self._actions.append(
            launch.actions.GroupAction(
                [action],
                condition=launch.conditions.IfCondition(launch.substitutions.PythonExpression(
                    ['"', self._selector, '"', f'=="{value}"'])
                ),
            )
        )

    @property
    def action(self) -> launch.actions.GroupAction:
        return launch.actions.GroupAction(
            self._actions
        )


def generate_launch_description():
    # Current path to the workspace
    entity_manager = LaunchArgument(
        name='entity_manager',
        choices=['dummy', 'hunav'],
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
        launch_entity_manager.action,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
