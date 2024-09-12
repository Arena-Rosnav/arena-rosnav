import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


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
    GZ_SIM_RESOURCE_PATHS = [
        os.path.join(workspace_root, 'src', 'gazebo', 'gz-sim', 'test', 'worlds', 'models'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'entities'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'worlds'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'gazebo_models')
    ]
    separator = ':'
    GZ_SIM_RESOURCE_PATHS_COMBINED = separator.join(GZ_SIM_RESOURCE_PATHS)
    
    # Update the environment
    os.environ['GZ_CONFIG_PATH'] = GZ_CONFIG_PATH
    os.environ['GZ_SIM_PHYSICS_ENGINE_PATH'] = GZ_SIM_PHYSICS_ENGINE_PATH
    os.environ['GZ_SIM_RESOURCE_PATH'] = GZ_SIM_RESOURCE_PATHS_COMBINED

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

    # model = launch.substitutions.LaunchConfiguration('model', default='jackal')
    # robot_path = PathJoinSubstitution([
    #     get_package_share_directory('arena_simulation_setup'),
    #     'entities', 
    #     'robots',
    #     model,
    #     'urdf',
    #     TextSubstitution(text='%s.gazebo' % model)
    # ])

    # Set the physics engine to dartsim
    physics_engine = 'gz-physics-dartsim'
    
    # Get the path to the YAML config file
    config_file = os.path.join(workspace_root, 'src', 'arena', 'arena-rosnav', 'arena_bringup', 'launch', 'testing', 'simulators', 'gazebo_bridge.yaml')

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    robot_model = launch.substitutions.LaunchConfiguration('model', default='jackal')  # Default world
    
    robot_desc = os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'entities', 'robots', str(robot_model), 'urdf', str(robot_model)+'.gazebo')
    
    
    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )    

    # Launch gz_sim.launch.py with arguments
    
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                gz_sim_launch_file
            ),
            launch_arguments={
                'gz_args': world_file,
                # 'robot': robot_path,
                # 'verbose': 'true',  # Optional for debugging
                # 'headless': launch.substitutions.LaunchConfiguration('headless'),
                'physics-engine': physics_engine
            }.items()
        ),
        bridge,
        robot_state_publisher
    ])
    return ld

def add_directories_recursively(root_dirs):
    all_dirs = []
    for root_dir in root_dirs:
        for dirpath, dirnames, filenames in os.walk(root_dir):
            all_dirs.append(dirpath)
    return all_dirs


if __name__ == '__main__':
    generate_launch_description()