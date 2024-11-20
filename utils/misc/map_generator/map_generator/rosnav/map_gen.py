from enum import Enum
from typing import Tuple

import numpy as np
import rospy
from map_generator.base_map_gen import BaseMapGenerator
from map_generator.constants import MapGenerators, MAP_GENERATOR_NS
from map_generator.factory import MapGeneratorFactory
from map_generator.rosnav.wrapper import create_indoor_map, create_outdoor_map


class MAP_TYPE(Enum):
    indoor = "indoor"
    outdoor = "outdoor"


@MapGeneratorFactory.register(MapGenerators.ROSNAV)
class RosnavMapGenerator(BaseMapGenerator):
    def __init__(
        self,
        height: int,
        width: int,
        map_type: str = "indoor",
        corridor_radius: int = 5,
        iterations: int = 100,
        obstacle_num: int = 10,
        obstacle_extra_radius: int = 1,
        *args,
        **kwargs,
    ):
        super().__init__(height, width, map_resolution=kwargs["map_resolution"])

        self.map_type = MAP_TYPE(map_type.lower())

        # indoor parameters
        self.iterations = iterations
        self.corridor_radius = corridor_radius

        # outdoor parameters
        self.obstacle_num = obstacle_num
        # extra radius for obstacles, middle point is the obstacle center
        self.obstacle_extra_radius = obstacle_extra_radius

    def update_params(
        self,
        height: int,
        width: int,
        map_res: float,
        map_type: str,
        iterations: int,
        corridor_radius: int,
        obstacle_num: int,
        obstacle_extra_radius: int,
    ):
        super().update_params(height, width, map_res)
        self.map_type = map_type

        # indoor params
        self.iterations, self.corridor_radius = iterations, corridor_radius
        # outdoor params
        self.obstacle_num, self.obstacle_extra_radius = (
            obstacle_num,
            obstacle_extra_radius,
        )

    def retrieve_params(self) -> Tuple[int, int, float, MAP_TYPE, int, int, int, int]:
        height, width, map_res = super().retrieve_params()
        map_type = rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/rosnav/map_type"), self.map_type
        )
        if type(map_type) == str:
            map_type = MAP_TYPE(map_type.lower())

        # indoor params
        corridor_radius = rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/rosnav/indoor/corridor_radius"),
            self.corridor_radius,
        )
        iterations = rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/rosnav/indoor/iterations"),
            self.iterations,
        )

        # outdoor params
        obstacle_num = rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/rosnav/outdoor/obstacle_num"),
            self.obstacle_num,
        )
        obstacle_extra_radius = rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/rosnav/outdoor/obstacle_extra_radius"),
            self.obstacle_extra_radius,
        )

        return (
            height,
            width,
            map_res,
            map_type,
            iterations,
            corridor_radius,
            obstacle_num,
            obstacle_extra_radius,
        )

    def generate_grid_map(self) -> np.ndarray:
        super().generate_grid_map()
        return (
            create_indoor_map(
                height=self.height,
                width=self.width,
                corridor_radius=self.corridor_radius,
                iterations=self.iterations,
            )
            if self.map_type in [MAP_TYPE.indoor, "indoor"]
            else create_outdoor_map(
                height=self.height,
                width=self.width,
                obstacle_number=self.obstacle_num,
                obstacle_extra_radius=self.obstacle_extra_radius,
            )
        )


def test():
    map_gen = RosnavMapGenerator(
        height=100, width=100, map_type="indoor", corridor_radius=5, iterations=100
    )
    grid_map = map_gen.generate_grid_map()


if __name__ == "__main__":
    test()
