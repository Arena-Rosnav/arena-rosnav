import itertools
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from arena_bringup.future import IfElseSubstitution, PythonExpression  # noqa
from arena_bringup.substitutions import LaunchArgument


def generate_launch_description():

    use_sim_time = LaunchArgument(
        "use_sim_time",
        default_value='True',
        description="Use simulation (Gazebo) clock if true",
    )

    world = LaunchArgument(
        "world",
        default_value='',
        description="World name",
    )

    headless = LaunchArgument(
        'headless',
    )

    # Set environment variables
    package_root = get_package_share_directory('arena_bringup')
    ss_root = get_package_share_directory('arena_simulation_setup')
    ss_models = os.path.join(ss_root, '../..', 'models')

    # Set paths for Gazebo, Physics Engine, and Resource

    # GZ_CONFIG_PATHS = [
    #     # os.path.join(get_package_share_directory('gz-sim8'), "gz"),
    #     # os.path.join(workspace_root, 'install', 'gz-tools2', 'share', 'gz'),
    # ]

    # GZ_SIM_PHYSICS_ENGINE_PATH = os.path.join(
    #     workspace_root, "build", "gz-physics7"
    # )

    staging_path = os.path.join(package_root, '..', 'staging')
    os.makedirs(staging_path, exist_ok=True)

    import subprocess
    subprocess.run(['ros2', 'run', 'arena_simulation_setup', 'model_staging', staging_path])

    GZ_SIM_RESOURCE_PATHS = [
        os.path.join(staging_path),
    ]

    deps_file = os.path.join(staging_path, 'deps')
    if os.path.isfile(deps_file):
        with open(deps_file) as f:
            deps = f.readlines()
            for package in itertools.chain(deps, ('arena_simulation_setup',)):
                try:
                    package_path = get_package_share_directory(package.strip())
                    GZ_SIM_RESOURCE_PATHS.append(os.path.join(package_path, '..'))
                except BaseException:
                    pass

    GZ_SIM_RESOURCE_PATHS = [os.path.normpath(path) for path in GZ_SIM_RESOURCE_PATHS]

    # GZ_CONFIG_PATH = ":".join(GZ_CONFIG_PATHS)
    GZ_CONFIG_PATH = "/usr/share/gz"

    for root, dirs, files in os.walk(os.path.join(ss_root, "gazebo_models")):
        for dir_name in dirs:
            if 'hospital' in dir_name.lower():
                GZ_SIM_RESOURCE_PATHS.append(os.path.join(root, dir_name))

    GZ_SIM_RESOURCE_PATHS_COMBINED = ":".join(GZ_SIM_RESOURCE_PATHS)

    # Update environment variables
    model_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if model_path:
        GZ_SIM_RESOURCE_PATHS_COMBINED = f"{model_path}:{GZ_SIM_RESOURCE_PATHS_COMBINED}"
    os.environ["GZ_SIM_RESOURCE_PATH"] = GZ_SIM_RESOURCE_PATHS_COMBINED
    os.environ["GAZEBO_MODEL_PATH"] = GZ_SIM_RESOURCE_PATHS_COMBINED
    # os.environ['GZ_CONFIG_PATH'] = GZ_CONFIG_PATH
    # os.environ["GZ_CONFIG_PATH"] = GZ_CONFIG_PATH
    # os.environ["GZ_SIM_PHYSICS_ENGINE_PATH"] = GZ_SIM_PHYSICS_ENGINE_PATH

    desired_world = PathJoinSubstitution(
        [
            ss_root,
            "worlds",
            world.substitution,
            "worlds",
            PythonExpression(['"', world.substitution, '.world"']),
        ]
    )

    world_path = IfElseSubstitution(
        condition=PythonExpression(['not os.path.isfile("', desired_world, '")'], python_modules=['os']),
        if_value=PathJoinSubstitution(
            [
                package_root,
                'configs',
                'gazebo',
                'empty.sdf',
            ]
        ),
        else_value=desired_world,
    )

    # Gazebo launch
    gz_sim_launch_file = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            "gz_args": [
                world_path,
                # " -v 4",
                " -r",
                " --render-engine ogre",
                IfElseSubstitution(
                    headless.substitution,
                    " -s",
                    "",
                ),
            ],
            "physics-engine": "gz-physics-dartsim",
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{
            **use_sim_time.dict
        }],
    )

    # Return the LaunchDescription with all the nodes/actions

    return LaunchDescription(
        [
            use_sim_time,
            world,
            headless,
            SetEnvironmentVariable("GZ_CONFIG_PATH", GZ_CONFIG_PATH),
            # SetEnvironmentVariable(
            #     "GZ_SIM_PHYSICS_ENGINE_PATH", GZ_SIM_PHYSICS_ENGINE_PATH
            # ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", GZ_SIM_RESOURCE_PATHS_COMBINED
            ),
            gazebo,
            # robot_state_publisher,
            # joint_state_publisher,
            # spawn_robot,
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(random_spawn_launch_file),
            #     condition=IfCondition(random_spawn_test),
            # ),
            clock_bridge,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
