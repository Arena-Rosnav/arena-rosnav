from map_generator.constants import (
    MapGenerators,
    MAP_GENERATOR_NS,
)
from typing import Tuple

import numpy as np
import rospy
from enum import Enum

from map_generator.base_map_gen import BaseMapGenerator
from map_generator.factory import MapGeneratorFactory
from .map_types.canteen import create_canteen_map
from .map_types.outdoor import create_outdoor_map
from .map_types.warehouse import create_warehouse_map


class MAP_TYPE(Enum):
    canteen = "canteen"
    outdoor = "outdoor"
    warehouse = "warehouse"


@MapGeneratorFactory.register(MapGenerators.ROSMAP)
class RosnavMapGenerator(BaseMapGenerator):
    def __init__(
        self,
        height: int,
        width: int,
        robot_infl_radius: float,
        map_resolution: float,
        map_type: str = "outdoor",
        obstacle_num: int = 10,
        obstacle_extra_radius: int = 1,
        chair_chance: float = 0.4,
        *args,
        **kwargs,
    ):
        super().__init__(height, width,
                         map_resolution=map_resolution)

        self.map_type = MAP_TYPE(map_type.lower())

        # parameters
        self.obstacle_num = obstacle_num
        self.map_resolution = map_resolution
        # extra radius for obstacles, middle point is the obstacle center
        self.obstacle_extra_radius = obstacle_extra_radius
        # the probability of each chair of 4 at a table
        self.chair_chance = chair_chance

    def update_params(
        self,
        height: int,
        width: int,
        map_res: float,
        map_type: str,
        obstacle_num: int,
        obstacle_extra_radius: int,
        chair_chance: float
    ):
        super().update_params(height, width, map_res)
        self.map_type = map_type

        # canteen params
        self.chair_chance = chair_chance

        # outdoor params
        self.obstacle_num, self.obstacle_extra_radius = (
            obstacle_num,
            obstacle_extra_radius,
        )

    def retrieve_params(self) -> Tuple[int, int, float, MAP_TYPE, int, int, float]:
        height, width, map_res = super().retrieve_params()
        map_type = rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/rosnav/map_type"), self.map_type
        )
        if type(map_type) == str:
            map_type = MAP_TYPE(map_type.lower())

        # params
        chair_chance = rospy.get_param(
            MAP_GENERATOR_NS("/algorithm_config/rosmap/canteen/chair_chance"),
            self.chair_chance
        )
        obstacle_num = rospy.get_param(
            MAP_GENERATOR_NS("/algorithm_config/rosmap/outdoor/obstacle_num"),
            self.obstacle_num
        )
        obstacle_extra_radius = rospy.get_param(
            MAP_GENERATOR_NS(
                "/algorithm_config/rosmap/outdoor/obstacle_extra_radius"),
            self.obstacle_extra_radius,
        )

        return (
            height,
            width,
            map_res,
            map_type,
            obstacle_num,
            obstacle_extra_radius,
            chair_chance
        )

    def generate_grid_map(self) -> (np.ndarray, dict):
        super().generate_grid_map()
        if self.map_type in [MAP_TYPE.canteen, "canteen"]:
            return create_canteen_map(
                height=self.height,
                width=self.width,
                obstacle_number=self.obstacle_num,
                obstacle_extra_radius=self.obstacle_extra_radius,
                chair_chance=self.chair_chance,
                map_resolution=self.map_resolution
            )
        elif self.map_type in [MAP_TYPE.outdoor, "outdoor"]:
            return create_outdoor_map(
                    height=self.height,
                    width=self.width,
                    obstacle_number=self.obstacle_num,
                    obstacle_extra_radius=self.obstacle_extra_radius,
                    map_resolution=self.map_resolution
                )
        elif self.map_type in [MAP_TYPE.warehouse, "warehouse"]:
            return create_warehouse_map(
                    height=self.height,
                    width=self.width,
                    obstacle_number=self.obstacle_num,
                    obstacle_extra_radius=self.obstacle_extra_radius,
                    map_resolution=self.map_resolution
                )


def test():
    map_gen = RosnavMapGenerator(
        height=100, width=100, map_type="canteen"
    )
    grid_map = map_gen.generate_grid_map()


if __name__ == "__main__":
    test()
