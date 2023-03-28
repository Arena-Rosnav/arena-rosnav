#!/usr/bin/env python3
import subprocess

import numpy as np

import rospy
import rospkg
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

import map_generator
from map_generator.constants import MAP_FOLDER_NAME
from map_generator.factory import MapGeneratorFactory
from map_generator.base_map_gen import BaseMapGenerator
from map_generator.utils.map import make_image
from map_generator.utils.general import load_config, get_config_path, get_rosnav_configs


class MapGeneratorNode:
    def __init__(self, map_generator: BaseMapGenerator) -> None:
        if not issubclass(type(map_generator), BaseMapGenerator):
            raise AttributeError(
                "'map_generator' must be a subclass of BaseMapGenerator"
            )

        # self._ns_prefix = f"/{ns}/" if ns else ""
        self.map_generator = map_generator
        self.train_mode = rospy.get_param("train_mode", False)

        self.map_path = rospkg.RosPack().get_path("arena-simulation-setup") + "/maps"

        # initialize occupancy grid
        self.occupancy_grid = OccupancyGrid()

        rospy.Subscriber("map", OccupancyGrid, self._get_occupancy_grid)
        rospy.Subscriber("request_new_map", String, self.callback_new_map)
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=1)

    def _get_occupancy_grid(self, occgrid_msg: OccupancyGrid):
        self.occupancy_grid = occgrid_msg

    def callback_new_map(self, msg: String):
        grid_map = self.map_generator.generate_grid_map()
        MapGeneratorNode.save_map(grid_map, self.map_path, MAP_FOLDER_NAME)

        self.occupancy_grid.data = MapGeneratorNode.preprocess_map_data(grid_map)

        self.map_pub.publish(self.occupancy_grid)

        if not self.train_mode:
            bashCommand = "rosservice call /move_base/clear_costmaps"
            subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)

        rospy.loginfo("New random map published and costmap cleared.")

    @staticmethod
    def preprocess_map_data(grid_map: np.ndarray) -> np.ndarray:
        # flip from [height, width] to [width, height]
        grid_map = np.flip(grid_map, axis=0)
        # map currently [0,1] 2D np array needs to be flattened for publishing OccupancyGrid.data
        return (grid_map * 100).flatten()

    @staticmethod
    def save_map(grid_map: np.ndarray, map_path: str, map_name: str):
        make_image(map=grid_map, map_name=map_name, dir_path=map_path)


def main():
    rospy.init_node("map_generator")

    # map_gen = MapGeneratorFactory.instantiate(
    #     "barn", 75, 75, fill_pct=0.2, smooth_iter=5
    # )

    cfg_path = get_config_path()
    cfg = load_config(cfg_path)

    map_properties = cfg["map_properties"]
    gen_configs = (
        cfg["generator_configs"]["barn"]
        if cfg["generator"] == "barn"
        else get_rosnav_configs(cfg)
    )

    map_gen = MapGeneratorFactory.instantiate(
        name=cfg["generator"],
        width=map_properties["width"],
        height=map_properties["height"],
        **gen_configs,
    )

    map_gen_node = MapGeneratorNode(map_generator=map_gen)
    rospy.spin()


if __name__ == "__main__":
    main()
