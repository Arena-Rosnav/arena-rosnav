#!/usr/bin/env python3
"""
Node that resets the map (amp folder: dynamic_map) to a blank state according to the parameters.
Then starts the map_server node.
"""
import rospy
import roslaunch

from map_generator.constants import MAP_FOLDER_NAME, ROSNAV_MAP_FOLDER, MAP_GENERATOR_NS
from map_generator.utils.map import create_empty_map, create_yaml_files
from map_generator.utils.general import load_map_generator_config


def start_map_server():
    """
    Starts the map server node.

    This function initializes the ROS node, creates a map server node, and launches it.
    It also handles the shutdown process when the ROS node is terminated.

    Args:
        None

    Returns:
        None
    """
    rospy.init_node("map_server_starter", anonymous=False)

    pkg = "map_server"
    executable = "map_server"
    args = rospy.get_param("map_path")
    remap_args = (
        None
        if rospy.get_param("single_env", True)
        else [("/clock", "/clock_simulation")]
    )

    node = roslaunch.core.Node(
        pkg,
        executable,
        args=args,
        remap_args=remap_args,
    )

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)

    while not rospy.is_shutdown():
        rospy.spin()

    process.stop()


def main():
    """
    Main function for map server.
    It loads the map generator configuration, creates an empty map,
    generates YAML files, and starts the map server.
    """
    if rospy.get_param("map_file", "") == MAP_FOLDER_NAME:
        cfg = load_map_generator_config()
        map_properties = rospy.get_param(
            MAP_GENERATOR_NS("map_properties"), cfg["map_properties"]
        )

        create_empty_map(
            height=map_properties["height"],
            width=map_properties["width"],
            map_name=MAP_FOLDER_NAME,
            dir_path=ROSNAV_MAP_FOLDER,
        )
        create_yaml_files(map_name=MAP_FOLDER_NAME, dir_path=ROSNAV_MAP_FOLDER)

        start_map_server()


if __name__ == "__main__":
    main()
