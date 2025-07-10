import os
import random
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_random_position():
    """Generate random positions within a specific range."""
    x = random.uniform(-5.0, 5.0)
    y = random.uniform(-5.0, 5.0)
    z = 0.0  # Assuming chairs are placed on the ground
    return x, y, z


def read_sdf_as_xml_string(sdf_file_path):
    """Read the SDF file and return its content as an XML string."""
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    return ET.tostring(root, encoding="unicode")


def generate_spawn_node(chair_index, chair_sdf_xml, chair_sdf_path, x, y, z):
    """Generate a node to spawn the chair model at the specified position."""
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": "default",
                "file": chair_sdf_path,
                #  'string': chair_sdf_xml,
                "name": f"chair_{chair_index}",
                "allow_renaming": False,
                "x": x,
                "y": y,
                "z": z,
            }
        ],
    )


def generate_launch_description():
    # Set the workspace root
    current_dir = os.path.abspath(__file__)
    workspace_root = current_dir
    while not workspace_root.endswith("arena4_ws"):
        workspace_root = os.path.dirname(workspace_root)

    if not workspace_root.endswith("arena4_ws"):
        raise ValueError(
            "Could not find the 'arena4_ws' directory in the current path."
        )

    # Path to the chair SDF file
    chair_sdf_path = os.path.join(
        workspace_root,
        "src",
        "arena",
        "arena_simulation_setup",
        "entities",
        "obstacles",
        "static",
        "chair",
        "sdf",
        "chair.sdf",
    )

    # Read the SDF file as an XML string
    chair_sdf_xml = read_sdf_as_xml_string(chair_sdf_path)

    # Launch Arguments
    num_chairs = LaunchConfiguration("num_chairs", default="5")

    # Generate spawn nodes for chairs
    spawn_chair_nodes = []
    for i in range(5):  # Generate up to 5 chairs
        x, y, z = generate_random_position()
        spawn_chair_nodes.append(
            LogInfo(msg=f"Spawning chair {i} at position ({x}, {y}, {z})")
        )
        spawn_chair_nodes.append(
            generate_spawn_node(i, chair_sdf_xml, chair_sdf_path, x, y, z)
        )

    # Return the LaunchDescription with all the nodes/actions
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_chairs",
                default_value="5",
                description="Number of chairs to spawn",
            ),
            *spawn_chair_nodes,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
