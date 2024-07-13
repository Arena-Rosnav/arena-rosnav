#!/usr/bin/env python3
"""
Node that resets the map (amp folder: dynamic_map) to a blank state according to the parameters.
Then starts the map_server node.
"""
import numpy as np
import rospy
import roslaunch

from map_generator.constants import MAP_FOLDER_NAME, ROSNAV_MAP_FOLDER, MAP_GENERATOR_NS
from map_generator.utils.map import MapPublisher
from map_generator.utils.general import load_map_generator_config

import std_msgs.msg

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

    pkg = "map_server"
    executable = "map_server"
    args = rospy.get_param("map_path")

    _is_multi_env = isinstance(rospy.get_param("num_envs", None), int)

    remap_args = None if not _is_multi_env else [("/clock", "/clock_simulation")]

    node = roslaunch.core.Node(
        pkg,
        executable,
        args=args,
        remap_args=remap_args,
    )

    launch = roslaunch.scriptapi.ROSLaunch()

    launch.start()

    process = launch.launch(node)

    rospy.set_param('map_server_node', node.name)

    while not rospy.is_shutdown():
        rospy.spin()

    launch.stop()


def main():
    """
    Main function for map server.
    It loads the map generator configuration, creates an empty map,
    generates YAML files, and starts the map server.
    """

    rospy.init_node("map_server_starter", anonymous=False)

    cfg = load_map_generator_config()
    map_properties = rospy.get_param(
        MAP_GENERATOR_NS("map_properties"), cfg["map_properties"]
    )

    resolution: float = map_properties['resolution']

    MapPublisher().publish_map(
        np.zeros((int(100/resolution),int(100/resolution))),
        {'resolution':resolution},
        {}
    )

    start_map_server()


if __name__ == "__main__":
    main()
