import os
import launch
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

    simulator = LaunchArgument(
        name='simulator',
        choices=['dummy', 'gazebo', 'isaac'],
    )

    use_sim_time = LaunchArgument(
        name='use_sim_time',
    )

    headless = LaunchArgument(
        name='headless',
        choices=['0', '1', '2'],
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
                'use_sim_time': use_sim_time.substitution,
                # 'headless': headless.substitution
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
        launch_simulator.action,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
