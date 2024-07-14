#!/usr/bin/env python3
import subprocess
from typing import List

import rospy
from map_generator.base_map_gen import BaseMapGenerator
from map_generator.constants import (
    PARAM_GENERATING,
    MapGenerators,
    MAP_GENERATOR_NS,
)
from map_generator.factory import MapGeneratorFactory
from map_generator.utils.general import (
    delete_distance_map,
    load_map_generator_config,
    load_robot_config,
)
from map_generator.utils.map import PARAM_MAP_PROPERTIES, MapPublisher
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

def get_generator_params(default_dict: dict) -> tuple:
    generator = rospy.get_param(
        MAP_GENERATOR_NS("algorithm"), default_dict["algorithm"]
    )
    map_properties = rospy.get_param(
        PARAM_MAP_PROPERTIES, default_dict["map_properties"]
    )
    gen_cfg = rospy.get_param(
        MAP_GENERATOR_NS("algorithm_config"), default_dict["algorithm_config"]
    )

    return generator, map_properties, gen_cfg


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

    Methods:createsnew_map(msg: String): Procedure to publish a new map.
        preprocess_map_data(grid_map: np.ndarray) -> np.ndarray: Preprocesses the grid map data.
        save_map(grid_map: np.ndarray, map_path: str, map_name: str): Saves the grid map as a PNG file.
        get_all_robot_topics() -> List[str]: Retrieves all robot namespaces.
    """

    def __init__(self):
        self._train_mode = rospy.get_param("train_mode", False)
        self._robot_infl_rad = load_robot_config(rospy.get_param("model"))[
            "robot_radius"
        ]

        delete_distance_map()

        rospy.Subscriber("/map", OccupancyGrid, self._get_occupancy_grid)
        rospy.Subscriber("/request_new_map", String, self._callback_new_map, queue_size=1)

        self._default_config = load_map_generator_config()
        self._robot_namespaces = MapGeneratorNode.get_all_robot_topics()
        self._map_generator = self._initialize_map_generator()

        self._map_publisher = MapPublisher()

    def _initialize_map_generator(self) -> BaseMapGenerator:
        self._generator_name, map_properties, gen_cfg = get_generator_params(
            self._default_config
        )

        return MapGeneratorFactory.instantiate(
            name=MapGenerators[self._generator_name.upper()],
            width=map_properties["width"],
            height=map_properties["height"],
            map_resolution=map_properties["resolution"],
            robot_infl_radius=self._robot_infl_rad * 1.75,
            **gen_cfg,
        )

    @property
    def generator_name(self) -> str:
        """Retrieves the map generator.

        Returns:
            str: The map generator.
        """
        return self._generator_name

    @property
    def occupancy_grid(self) -> OccupancyGrid:
        """Retrieves the most recent occupancy grid.

        Returns:
            OccupancyGrid: The most recent occupancy grid.
        """
        return self._occupancy_grid

    def _get_occupancy_grid(self, occgrid_msg: OccupancyGrid):
        """Saves the most recent occupancy grid.

        Args:
            occgrid_msg (OccupancyGrid): Message containing the current occupancy grid.
        """
        self._occupancy_grid = occgrid_msg

    def _callback_new_map(self, msg: String):
        """Procedure to publish a new map.

        Steps on receiving a message on "/request_new_map":
            1. Generates a grid map with the dedicated algorithm.
            2. Saves the grid map as a PNG to be loaded by Flatland.
            3. Publishes the new map.

        Args:
            msg (String): Empty message as the message is not used.
        """
        while rospy.get_param(PARAM_GENERATING, False):
            rospy.sleep(.1)
        rospy.set_param(PARAM_GENERATING, True)

        try:
            if self.generator_name != rospy.get_param(MAP_GENERATOR_NS("algorithm")):
                self._map_generator = self._initialize_map_generator()

            grid_map, extras = self._map_generator.generate_map()

            map_properties = dict(
                resolution = self._map_generator.map_resolution 
            )

            self._map_publisher.publish_map(grid_map, map_properties, extras)
            
            if not self._train_mode:
                for robot_ns in self._robot_namespaces:
                    bashCommand = f"rosservice call /{robot_ns}/move_base/clear_costmaps"
                    subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)

            rospy.loginfo("New random map published and costmap cleared.")

        finally:
            rospy.set_param(PARAM_GENERATING, False)
            self._map_generator.idle()

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
    map_gen_node = MapGeneratorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
