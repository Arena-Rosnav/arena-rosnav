import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    # Set environment variables
    GZ_CONFIG_PATH = "/root/arena4_ws/install/gz-sim7/share/gz"  # Set the path directly
    GZ_SIM_PHYSICS_ENGINE_PATH = "/root/arena4_ws/build/gz-physics6"
    env = os.environ.copy()
    env['GZ_CONFIG_PATH'] = GZ_CONFIG_PATH
    env['GZ_SIM_PHYSICS_ENGINE_PATH'] = GZ_SIM_PHYSICS_ENGINE_PATH
    
    
    # Get the path to the gz_sim.launch.py file
    gz_sim_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    # Define paths to your world and robot SDF files (using defaults if not provided)
    world_file = launch.substitutions.LaunchConfiguration('world_file', default='empty')  # Default world
    world_path = PathJoinSubstitution([
        get_package_share_directory('arena_simulation_setup'),
        'worlds',
        world_file,
        '.sdf'
    ])

    model = launch.substitutions.LaunchConfiguration('model', default='jackal')  
    robot_sdf_path = PathJoinSubstitution([
        get_package_share_directory('arena_simulation_setup'),
        'entities', 
        'robots',
        model,
        'urdf',
        TextSubstitution(text='%s.gazebo' % model) # Correct way to use TextSubstitution
    ])
    # Launch gz_sim.launch.py with arguments
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                gz_sim_launch_file
            ),
            launch_arguments={
                'world': world_path,
                'sdf': robot_sdf_path,
                'verbose': 'true',  # Optional for debugging
                'headless': launch.substitutions.LaunchConfiguration('headless')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()