import os
import random
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_random_position():
    """Generate random positions within a specific range."""
    x = random.uniform(-5.0, 5.0)
    y = random.uniform(-5.0, 5.0)
    z = 0.0  # Assuming boxes are placed on the ground
    return x, y, z

def read_sdf_as_xml_string(sdf_file_path):
    """Read the SDF file and return its content as an XML string."""
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    return ET.tostring(root, encoding='unicode')

def generate_spawn_node(box_index, box_sdf_xml, x, y, z):
    """Generate a node to spawn the box model at the specified position."""
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', box_sdf_xml,
            '-name', f'box_{box_index}',
            '-x', str(x),
            '-y', str(y),
            '-z', str(z)
        ],
    )

def generate_launch_description():
    # Set the workspace root
    current_dir = os.path.abspath(__file__)
    workspace_root = current_dir
    while not workspace_root.endswith('arena4_ws'):
        workspace_root = os.path.dirname(workspace_root)

    if not workspace_root.endswith('arena4_ws'):
        raise ValueError("Could not find the 'arena4_ws' directory in the current path.")

    # Path to the box SDF file
    box_sdf_path = os.path.join(
        workspace_root, 'src', 'gazebo', 'sdformat', 'test', 'integration', 'model', 'box', 'model.sdf'
    )

    # Read the SDF file as an XML string
    box_sdf_xml = read_sdf_as_xml_string(box_sdf_path)

    # Launch Arguments
    num_boxes = LaunchConfiguration('num_boxes', default='5')

    # Generate spawn nodes for boxes
    spawn_box_nodes = []
    for i in range(5):  # Generate up to 5 boxes
        x, y, z = generate_random_position()
        spawn_box_nodes.append(LogInfo(msg=f'Spawning box {i} at position ({x}, {y}, {z})'))
        spawn_box_nodes.append(generate_spawn_node(i, box_sdf_xml, x, y, z))

    # Return the LaunchDescription with all the nodes/actions
    return LaunchDescription([
        DeclareLaunchArgument('num_boxes', default_value='5', description='Number of boxes to spawn'),
        *spawn_box_nodes
    ])

if __name__ == '__main__':
    generate_launch_description()