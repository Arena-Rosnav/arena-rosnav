import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
import xacro

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
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'gazebo_models'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'gazebo_models', 'Cafe table', 'materials', 'textures'),
        os.path.join(workspace_root, 'src', 'deps')
    ]
    GZ_SIM_RESOURCE_PATHS_COMBINED = ':'.join(GZ_SIM_RESOURCE_PATHS)
    
    # Update the environment
    os.environ['GZ_CONFIG_PATH'] = GZ_CONFIG_PATH
    os.environ['GZ_SIM_PHYSICS_ENGINE_PATH'] = GZ_SIM_PHYSICS_ENGINE_PATH
    os.environ['GZ_SIM_RESOURCE_PATH'] = GZ_SIM_RESOURCE_PATHS_COMBINED

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    robot_model = LaunchConfiguration('model')

    # Gazebo launch
    gz_sim_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            'gz_args': [world_file, ' -v 4', ' -r'],
            'physics-engine': 'gz-physics-dartsim'
        }.items()
    )

    # Robot description
    robot_desc_path = os.path.join(
        workspace_root,
        'src', 'arena', 'simulation-setup', 'entities', 'robots',
        'jackal', 'urdf',
        'jackal' + '.urdf.xacro'
    )

    # Use xacro to process the robot description file
    doc = xacro.process_file(robot_desc_path, mappings={'use_sim': 'true'})
    robot_description = doc.toprettyxml(indent='  ')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
        ]
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description,
            '-name', robot_model,
            '-allow_renaming', 'false'
        ],
    )

    # Bridge
    config_file = os.path.join(workspace_root, 'src', 'arena', 'arena-rosnav', 'arena_bringup', 'launch', 'testing', 'simulators', 'gazebo_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('world_file', default_value='empty.sdf', description='World file name'),
        DeclareLaunchArgument('model', default_value='jackal', description='Robot model name'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])

def add_directories_recursively(root_dirs):
    all_dirs = []
    for root_dir in root_dirs:
        for dirpath, dirnames, filenames in os.walk(root_dir):
            all_dirs.append(dirpath)
    return all_dirs


if __name__ == '__main__':
    generate_launch_description()