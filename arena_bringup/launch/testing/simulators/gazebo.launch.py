import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    # Set environment variables
    current_dir = os.path.abspath(__file__)
    workspace_root = current_dir

    while not workspace_root.endswith('arena4_ws'):
        workspace_root = os.path.dirname(workspace_root)

    if not workspace_root.endswith('arena4_ws'):
        raise ValueError("Could not find the 'arena4_ws' directory in the current path.")
    GZ_CONFIG_PATH = os.path.join(workspace_root, 'install', 'gz-sim7', 'share', 'gz')
    GZ_SIM_PHYSICS_ENGINE_PATH = os.path.join(workspace_root, 'build', 'gz-physics6')
    GZ_SIM_RESOURCE_PATH = os.path.join(workspace_root, 'src', 'gazebo', 'gz-sim', 'test', 'worlds', 'models')
    # Update the environment
    os.environ['GZ_CONFIG_PATH'] = GZ_CONFIG_PATH
    os.environ['GZ_SIM_PHYSICS_ENGINE_PATH'] = GZ_SIM_PHYSICS_ENGINE_PATH
    os.environ['GZ_SIM_RESOURCE_PATH'] = GZ_SIM_RESOURCE_PATH

    # debug
    # print(f"GZ_CONFIG_PATH: {os.environ.get('GZ_CONFIG_PATH')}")
    # print(f"GZ_SIM_PHYSICS_ENGINE_PATH: {os.environ.get('GZ_SIM_PHYSICS_ENGINE_PATH')}")

    # Get the path to the gz_sim.launch.py file
    gz_sim_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )
    # Define paths to your world SDF and robot URDF/gazebo files (using defaults if not provided)
    world_file = launch.substitutions.LaunchConfiguration('world_file', default='empty.sdf')  # Default world

    model = launch.substitutions.LaunchConfiguration('model', default='jackal')
    robot_path = PathJoinSubstitution([
        get_package_share_directory('arena_simulation_setup'),
        'entities', 
        'robots',
        model,
        'urdf',
        TextSubstitution(text='%s.gazebo' % model)
    ])

    # Set the physics engine to Bullet Featherstone
    physics_engine = 'gz-physics-dartsim'

    # Launch gz_sim.launch.py with arguments
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                gz_sim_launch_file
            ),
            launch_arguments={
                'gz_args': world_file,
                'robot': robot_path,
                # 'verbose': 'true',  # Optional for debugging
                # 'headless': launch.substitutions.LaunchConfiguration('headless'),
                'physics-engine': physics_engine
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()