import os
import typing

import launch
from ament_index_python.packages import get_package_share_directory

import launch_ros.actions

from arena_bringup.substitutions import LaunchArgument
from arena_bringup.future import PythonExpression
from arena_bringup.actions import IsolatedGroupAction


def generate_launch_description():

    bringup_dir = get_package_share_directory('arena_bringup')

    ld_items = []
    LaunchArgument.auto_append(ld_items)

    # desised_resets = LaunchArgument(
    #     name='desired_resets',
    #     default_value='10'
    # )
    # timeout = LaunchArgument(
    #     name='timeout',
    #     default_value=''
    # )
    robot = LaunchArgument(
        name='robot',
        default_value='jackal',
        description='robot model type [burger, jackal, ridgeback, agvota, rto, ...]'
    )
    inter_planner = LaunchArgument(
        name='inter_planner',
        default_value='navigate_w_replanning_time',
        description='inter planner type (Behavior Tree)'
    )
    local_planner = LaunchArgument(
        name='local_planner',
        default_value='dwb',
        description='local planner type [teb, dwa, mpc, rlca, arena, rosnav, cohan]'
    )
    global_planner = LaunchArgument(
        name='global_planner',
        default_value='navfn',
        description='global planner type [navfn]'
    )
    simulator = LaunchArgument(
        name='simulator',
        default_value='dummy',
    )
    headless = LaunchArgument(
        name='headless',
        default_value='0',
        choices=['-1', '0', '1', '2'],
        description='-1 = show all environments, 0 = show all, 1 = show only rviz, 2 = show nothing'
    )
    entity_manager = LaunchArgument(
        name='entity_manager',
        default_value='dummy',
    )
    # sfm = LaunchArgument(
    #     name='sfm',
    #     default_value='',
    #     description='sfm for crowdsim [passthrough, pysocial]'
    # )
    complexity = LaunchArgument(
        name='complexity',
        default_value='1',
        description='1 = Map known, Position known; 2 = Map known, Position unknown (AMCL); 3 = Map unknown, Position unknown (SLAM)'
    )
    agent_name = LaunchArgument(
        name='agent_name',
        default_value=robot.substitution,
        description='DRL agent name to be deployed'
    ),
    record_data_dir = LaunchArgument(
        name='record_data_dir',
        default_value=''
    )
    tm_robots = LaunchArgument(
        name='tm_robots',
        default_value='explore'
    )
    tm_obstacles = LaunchArgument(
        name='tm_obstacles',
        default_value='random'
    )
    tm_modules = LaunchArgument(
        name='tm_modules',
        default_value='rviz_ui'  # TODO breaks launch if empty
    )
    show_rviz = LaunchArgument(
        name='show_rviz',
        default_value='true',
        description='Enables rviz in gazebo'
    )
    world = LaunchArgument(
        name='world',
        default_value='map_empty',
        description='world to load'
    )
    use_sim_time = LaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    env_n = LaunchArgument(
        name='env_n',
        default_value='1',
        description='Number of environments to spawn within simulator'
    )
    env_d = LaunchArgument(
        name='env_d',
        default_value='50',
        description='space between environments'
    )

    def create_task_generator(
        headlessness,
        namespace: str,
        prefix: str,
        reference: typing.List[float]
    ):
        return IsolatedGroupAction([
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('task_generator'),
                        'launch/task_generator.launch.py'
                    )
                ),
                launch_arguments={
                    **simulator.dict,
                    **entity_manager.dict,
                    **tm_obstacles.dict,
                    **tm_robots.dict,
                    **tm_modules.dict,
                    **robot.dict,
                    **inter_planner.dict,
                    **local_planner.dict,
                    **global_planner.dict,
                    **world.dict,
                    **record_data_dir.dict,
                    'namespace': namespace,
                    'headless': headlessness,
                    'reference': str(reference),
                    'prefix': prefix,
                    'parameter_file': os.path.join(get_package_share_directory('arena_bringup'), 'configs', 'task_generator.yaml'),
                }.items(),
            )
        ])

    def create_task_generators(
        context: launch.LaunchContext,
        *,
        n_substitution: launch.SomeSubstitutionsType,
        d_substitution: launch.SomeSubstitutionsType,
    ) -> typing.Optional[typing.List[launch.LaunchDescriptionEntity]]:

        n = launch.utilities.type_utils.perform_typed_substitution(
            context,
            launch.utilities.normalize_to_list_of_substitutions([n_substitution]),
            int,
        )
        d = launch.utilities.type_utils.perform_typed_substitution(
            context,
            launch.utilities.normalize_to_list_of_substitutions([d_substitution]),
            float,
        )

        if n < 1:
            return None

        task_generators = []
        base_namespace = 'task_generator_node'
        base_prefix = 'env'
        references = snail_grid(d)

        # first task generator
        task_generators.append(
            create_task_generator(
                headlessness=PythonExpression([headless.substitution, '>1']),
                namespace=base_namespace + ('_0' if n > 1 else ''),
                prefix=(base_prefix + '0_') if n > 1 else '',
                reference=list(next(references))
            )
        )

        # all following ones
        for i in range(1, n):
            task_generators.append(
                create_task_generator(
                    headlessness=PythonExpression([headless.substitution, '>-1']),
                    namespace=base_namespace + '_' + str(i),
                    prefix=base_prefix + str(i) + '_',
                    reference=list(next(references))
                )
            )

        return task_generators

    launch_task_generators = launch.actions.OpaqueFunction(
        function=create_task_generators,
        kwargs={
            'n_substitution': env_n.substitution,
            'd_substitution': env_d.substitution,
        },
    )

    launch_simulator = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch/shared/simulator/simulator.launch.py')
        ),
        launch_arguments={
            **use_sim_time.dict,
            **simulator.dict,
            **world.dict,
            'headless': PythonExpression([headless.substitution, '>0']),
        }.items(),
    )

    launch_entity_manager = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch/shared/entity_manager/entity_manager.launch.py')
        ),
        launch_arguments={
            **entity_manager.dict,
            **entity_manager.dict,
        }.items()
    )

    ld = launch.LaunchDescription([
        *ld_items,
        launch_task_generators,
        IsolatedGroupAction([launch_simulator]),
        IsolatedGroupAction([launch_entity_manager]),
    ])
    return ld


def snail_grid(d: float, initial=None):
    if initial is None:
        initial = (0, 0)
    x, y = map(float, initial)

    step: int = 0
    while True:
        yield x, y

        for _ in range(step):
            y -= d
            yield x, y

        for _ in range(step):
            x -= d
            yield x, y

        for _ in range(step):
            y += d
            yield x, y

        for _ in range(step):
            x += d
            yield x, y

        x += d
        y += d
        step += 2


if __name__ == '__main__':
    generate_launch_description()
