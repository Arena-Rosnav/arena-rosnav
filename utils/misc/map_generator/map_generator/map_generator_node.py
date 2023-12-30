#!/usr/bin/env python3
import subprocess
from typing import List

import map_generator
import numpy as np
import rospy
from map_generator.base_map_gen import BaseMapGenerator
from map_generator.constants import MAP_FOLDER_NAME, ROSNAV_MAP_FOLDER
from map_generator.factory import MapGeneratorFactory
from map_generator.utils.general import (
    delete_distance_map,
    get_rosnav_configs,
    load_map_generator_config,
    load_robot_config,
)
from map_generator.utils.map import make_image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class MapGeneratorNode:
    """Node responsible for generating and publishing maps.

    This class initializes a map generator and subscribes to the "/map" and "/request_new_map" topics.
    It generates a grid map using the map generator, saves it as a PNG file, and publishes the new map.
    If the train mode is disabled, it also clears the costmaps for all robot namespaces.

    Attributes:
        map_generator (BaseMapGenerator): The map generator used to generate the grid map.
        train_mode (bool): Flag indicating whether the node is in train mode.
        occupancy_grid (OccupancyGrid): The most recent occupancy grid.
        map_pub (rospy.Publisher): Publisher for the "/map" topic.
        robot_namespaces (List[str]): List of all robot namespaces.

    Methods:
        _get_occupancy_grid(occgrid_msg: OccupancyGrid): Saves the most recent occupancy grid.
        callback_new_map(msg: String): Procedure to publish a new map.
        preprocess_map_data(grid_map: np.ndarray) -> np.ndarray: Preprocesses the grid map data.
        save_map(grid_map: np.ndarray, map_path: str, map_name: str): Saves the grid map as a PNG file.
        get_all_robot_topics() -> List[str]: Retrieves all robot namespaces.
    """

    def __init__(self, map_generator_obj: BaseMapGenerator):
        if not issubclass(type(map_generator_obj), BaseMapGenerator):
            raise AttributeError(
                "'map_generator' must be a subclass of BaseMapGenerator"
            )

        # self._ns_prefix = f"/{ns}/" if ns else ""
        self.map_generator = map_generator_obj
        self.train_mode = rospy.get_param("train_mode", False)

        # initialize occupancy grid
        self.occupancy_grid = OccupancyGrid()

        delete_distance_map()

        rospy.Subscriber("/map", OccupancyGrid, self._get_occupancy_grid)
        rospy.Subscriber("/request_new_map", String, self.callback_new_map)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)

        self.robot_namespaces = MapGeneratorNode.get_all_robot_topics()

    def _get_occupancy_grid(self, occgrid_msg: OccupancyGrid):
        """Saves the most recent occupancy grid.

        Args:
            occgrid_msg (OccupancyGrid): Message containing the current occupancy grid.
        """
        self.occupancy_grid = occgrid_msg

    def callback_new_map(self, msg: String):
        """Procedure to publish a new map.

        Steps on receiving a message on "/request_new_map":
            1. Generates a grid map with the dedicated algorithm.
            2. Saves the grid map as a PNG to be loaded by Flatland.
            3. Publishes the new map.

        Args:
            msg (String): Empty message as the message is not used.
        """
        grid_map = self.map_generator.generate_grid_map()
        MapGeneratorNode.save_map(grid_map, ROSNAV_MAP_FOLDER, MAP_FOLDER_NAME)

        self.occupancy_grid.data = MapGeneratorNode.preprocess_map_data(grid_map)

        self.map_pub.publish(self.occupancy_grid)

        if not self.train_mode:
            for robot_ns in self.robot_namespaces:
                bashCommand = f"rosservice call /{robot_ns}/move_base/clear_costmaps"
                subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)

        rospy.loginfo("New random map published and costmap cleared.")

    @staticmethod
    def preprocess_map_data(grid_map: np.ndarray) -> np.ndarray:
        """Preprocesses the grid map data.

        Flips the grid map from [height, width] to [width, height] and flattens it for publishing OccupancyGrid.data.

        Args:
            grid_map (np.ndarray): The grid map to be preprocessed.

        Returns:
            np.ndarray: The preprocessed grid map data.
        """
        # flip from [height, width] to [width, height]
        grid_map = np.flip(grid_map, axis=0)
        # map currently [0,1] 2D np array needs to be flattened for publishing OccupancyGrid.data
        return (grid_map * 100).flatten().astype(np.int8)

    @staticmethod
    def save_map(grid_map: np.ndarray, map_path: str, map_name: str):
        """Saves the grid map as a PNG file.

        Args:
            grid_map (np.ndarray): The grid map to be saved.
            map_path (str): The path to save the map.
            map_name (str): The name of the map file.
        """
        make_image(map=grid_map, map_name=map_name, dir_path=map_path)

    @staticmethod
    def get_all_robot_topics() -> List[str]:
        """Retrieves all robot namespaces.

        Returns:
            List[str]: List of all robot namespaces.
        """
        robot_model = rospy.get_param("model")
        all_topics = rospy.get_published_topics()

        return [topic for topic, _ in all_topics if robot_model in topic.split("/")[0]]


def main():
    rospy.init_node("map_generator")

    cfg = load_map_generator_config()
    generator = rospy.get_param("generator", cfg["generator"])
    map_properties = rospy.get_param("map_properties", cfg["map_properties"])
    gen_cfg = rospy.get_param("generator_configs", cfg["generator_configs"])

    gen_configs = gen_cfg[generator.lower()]

    robot_infl_rad = load_robot_config(rospy.get_param("model"))["robot_radius"]

    map_gen = MapGeneratorFactory.instantiate(
        name=generator.lower(),
        width=map_properties["width"],
        height=map_properties["height"],
        map_resolution=map_properties["resolution"],
        robot_infl_radius=robot_infl_rad * 1.75,
        **gen_configs,
    )

    map_gen_node = MapGeneratorNode(map_generator_obj=map_gen)
    rospy.spin()


if __name__ == "__main__":
    main()
