import os
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import yaml
from ament_index_python.packages import get_package_share_directory

def get_workspace_root():
    """Get the workspace root directory."""
    current_dir = os.path.abspath(__file__)
    workspace_root = current_dir
    while not workspace_root.endswith('arena4_ws'):
        workspace_root = os.path.dirname(workspace_root)

    if not workspace_root.endswith('arena4_ws'):
        raise ValueError("Could not find the 'arena4_ws' directory in the current path.")
        
    return workspace_root

def load_agent_config() -> dict:
    """Load agent configurations from default.yaml."""
    # Get config file path
    config_path = os.path.join(
        get_workspace_root(),
        'src/arena/arena-rosnav/arena_bringup/configs/hunav_agents',
        'default.yaml'
    )
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        return config['hunav_loader']['ros__parameters']

def create_sdf_string(agent_name: str, agent_config: dict) -> str:
    """Create SDF string for an agent."""
    # Define skin types mapping
    SKIN_TYPES = {
        0: 'elegant_man.dae',
        1: 'casual_man.dae',
        2: 'elegant_woman.dae',
        3: 'regular_man.dae',
        4: 'worker_man.dae',
        5: 'walk.dae'
    }
    
    skin_type = SKIN_TYPES.get(agent_config.get('skin', 0), 'casual_man.dae')
    
    # Get workspace root and construct correct mesh path
    workspace_root = get_workspace_root()
    mesh_path = os.path.join(
        workspace_root,
        'src/deps/hunav_gazebo_wrapper/media/models',
        skin_type
    )
    
    print(f"Using mesh path: {mesh_path}")
    
    # Height adjustments
    height_adjustments = {
        'elegant_man.dae': 0.96,
        'casual_man.dae': 0.97,
        'elegant_woman.dae': 0.93,
        'regular_man.dae': 0.93,
        'worker_man.dae': 0.97
    }
    z_pos = height_adjustments.get(skin_type, 1.0)
    
    # Get position from config
    init_pose = agent_config.get('init_pose', {})
    x = init_pose.get('x', 0.0)
    y = init_pose.get('y', 0.0)
    yaw = init_pose.get('h', 0.0)
    
    sdf = f"""<?xml version="1.0" ?>
    <sdf version="1.6">
        <model name="{agent_name}">
            <static>false</static>
            <pose>{x} {y} {z_pos} 0 0 {yaw}</pose>
            <link name="link">
                <visual name="visual">
                    <geometry>
                        <mesh>
                            <uri>file://{mesh_path}</uri>
                        </mesh>
                    </geometry>
                </visual>
                <collision name="collision">
                    <geometry>
                        <cylinder>
                            <radius>0.3</radius>
                            <length>1.7</length>
                        </cylinder>
                    </geometry>
                </collision>
            </link>
        </model>
    </sdf>"""
    return sdf

def generate_spawn_node(agent_name: str, sdf_string: str, position):
    """Generate a node to spawn the agent model at the specified position."""
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', sdf_string,
            '-name', agent_name,
            '-x', str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2])
        ],
    )

def generate_launch_description():
    # Get agent configurations
    agent_configs = load_agent_config()
    
    # Generate spawn nodes for agents
    spawn_nodes = []
    
    for agent_name in agent_configs.get('agents', []):
        agent_config = agent_configs[agent_name]
        
        # Get position from config
        init_pose = agent_config.get('init_pose', {})
        position = (
            init_pose.get('x', 0.0),
            init_pose.get('y', 0.0),
            init_pose.get('z', 1.0)
        )
        
        # Generate SDF string
        sdf_string = create_sdf_string(agent_name, agent_config)
        
        # Create spawn logs and nodes
        spawn_nodes.append(
            LogInfo(msg=f'Spawning pedestrian {agent_name} at position {position}')
        )
        spawn_nodes.append(
            generate_spawn_node(agent_name, sdf_string, position)
        )

    # Return the LaunchDescription with all spawning actions
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        *spawn_nodes
    ])

if __name__ == '__main__':
    generate_launch_description()