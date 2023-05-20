#!/usr/bin/env python3
import subprocess

import numpy as np

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

import map_generator
from map_generator.base_map_gen import BaseMapGenerator
from map_generator.constants import MAP_FOLDER_NAME, ROSNAV_MAP_FOLDER
from map_generator.factory import MapGeneratorFactory
from map_generator.utils.map import make_image
from map_generator.utils.general import (
    load_robot_config,
    load_map_generator_config,
    get_rosnav_configs,
    delete_distance_map,
)


class MapGeneratorNode:
    def __init__(self, map_generator: BaseMapGenerator) -> None:
        if not issubclass(type(map_generator), BaseMapGenerator):
            raise AttributeError(
                "'map_generator' must be a subclass of BaseMapGenerator"
            )

        # self._ns_prefix = f"/{ns}/" if ns else ""
        self.map_generator = map_generator
        self.train_mode = rospy.get_param("train_mode", False)

        # initialize occupancy grid
        self.occupancy_grid = OccupancyGrid()

        delete_distance_map()

        rospy.Subscriber("/map", OccupancyGrid, self._get_occupancy_grid)
        rospy.Subscriber("/request_new_map", String, self.callback_new_map)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)

        self.robot_namespaces = MapGeneratorNode.get_all_robot_topics()

    def _get_occupancy_grid(self, occgrid_msg: OccupancyGrid):
        self.occupancy_grid = occgrid_msg

    def callback_new_map(self, msg: String):
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
        # flip from [height, width] to [width, height]
        grid_map = np.flip(grid_map, axis=0)
        # map currently [0,1] 2D np array needs to be flattened for publishing OccupancyGrid.data
        return (grid_map * 100).flatten().astype(np.int8)

    @staticmethod
    def save_map(grid_map: np.ndarray, map_path: str, map_name: str):
        make_image(map=grid_map, map_name=map_name, dir_path=map_path)

    @staticmethod
    def get_all_robot_topics():
        robot_model = rospy.get_param("model")
        all_topics = rospy.get_published_topics()

        return [topic for topic, _ in all_topics if robot_model in topic.split("/")[0]]


def main():
    rospy.init_node("map_generator")

    cfg = load_map_generator_config()
    generator = rospy.get_param("generator", cfg["generator"])
    map_properties = rospy.get_param("map_properties", cfg["map_properties"])
    gen_cfg = rospy.get_param("generator_configs", cfg["generator_configs"])

    gen_configs = (
        gen_cfg["barn"] if generator.lower() == "barn" else get_rosnav_configs(gen_cfg)
    )

    robot_infl_rad = load_robot_config(rospy.get_param("model"))["robot_radius"]

    map_gen = MapGeneratorFactory.instantiate(
        name=generator.lower(),
        width=map_properties["width"],
        height=map_properties["height"],
        map_resolution=map_properties["resolution"],
        robot_infl_radius=robot_infl_rad * 1.75,
        **gen_configs,
    )

    map_gen_node = MapGeneratorNode(map_generator=map_gen)
    rospy.spin()


if __name__ == "__main__":
    main()
