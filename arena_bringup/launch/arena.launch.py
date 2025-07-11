import os
import typing

import launch
from ament_index_python.packages import get_package_share_directory
from arena_bringup.actions import IsolatedGroupAction
from arena_bringup.extensions.NodeLogLevelExtension import SetGlobalLogLevelAction
from arena_bringup.future import IfElseSubstitution, PythonExpression
from arena_bringup.substitutions import LaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution

import launch_ros.actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('arena_bringup')

    ld_items = []
    LaunchArgument.auto_append(ld_items)

    log_level = LaunchArgument(
        name='log_level',
        default_value='warn',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
        description='Set the log level for all nodes'
    )

    robot = LaunchArgument(
        name='robot',
        default_value='jackal',
        description='robot model type'
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
    sim = LaunchArgument(
        name='sim',
        default_value='dummy',  # todo select first installed simulator
    )
    headless = LaunchArgument(
        name='headless',
        default_value='0',
        choices=['-1', '0', '1', '2'],
        description='-1 = show all environments, 0 = show all, 1 = show only rviz, 2 = show nothing'
    )
    human = LaunchArgument(
        name='human',
        description='human simulator to use',
        default_value=PythonExpression([str({"gazebo": "hunav", "isaac": "hunav"}), '.get("', sim.substitution, '", "dummy")']),
    )
    complexity = LaunchArgument(
        name='complexity',
        default_value='1',
        description='1 = Map known, Position known; 2 = Map known, Position unknown (AMCL); 3 = Map unknown, Position unknown (SLAM)'
    )
    agent_name = LaunchArgument(
        name='agent_name',
        default_value=robot.substitution,
        description='DRL agent name to be deployed'
    )
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

        # Log env_n value
        launch.actions.LogInfo(
            msg=[
                TextSubstitution(text="env_n value: "),
                TextSubstitution(text=str(n))
            ]
        ).execute(context)

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
                namespace=base_namespace,
                prefix='',
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

        # Log total task generators
        launch.actions.LogInfo(
            msg=[
                TextSubstitution(text="Total task_generator nodes spawned: "),
                TextSubstitution(text=str(len(task_generators)))
            ]
        ).execute(context)

        return task_generators

    def create_task_generator(
        headlessness,
        namespace: str,
        prefix: str,
        reference: typing.List[float]
    ):
        return IsolatedGroupAction([
            LogInfo(msg=[
                TextSubstitution(text="Spawning task_generator with namespace: "),
                TextSubstitution(text=namespace)
            ]),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch/simulator/human/human.launch.py')
                ),
                launch_arguments={
                    'simulator': human.substitution,
                    'namespace': namespace,
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('task_generator'),
                        'launch/task_generator.launch.py'
                    )
                ),
                launch_arguments={
                    **sim.dict,
                    **human.dict,
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

    launch_task_generators = launch.actions.OpaqueFunction(
        function=create_task_generators,
        kwargs={
            'n_substitution': env_n.substitution,
            'd_substitution': env_d.substitution,
        },
    )

    launch_simulator = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch/simulator/sim/sim.launch.py')
        ),
        launch_arguments={
            **use_sim_time.dict,
            'simulator': sim.substitution,
            **world.dict,
            'headless': PythonExpression([headless.substitution, '>0']),
        }.items(),
    )

    world_generator_node = launch_ros.actions.Node(
        package='arena_simulation_setup',
        executable='world_generator',
        name='world_generator',
        output='screen',
    )

    ld = launch.LaunchDescription([
        *ld_items,
        LogInfo(
            msg=[
                TextSubstitution(text="Starting arena bringup with env_n="),
                env_n.substitution,
                TextSubstitution(text=" task_generator_node(s)")
            ]
        ),
        SetGlobalLogLevelAction(log_level.substitution),
        launch_task_generators,
        IsolatedGroupAction([launch_simulator]),
        world_generator_node,
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
