import numpy as np
from enum import Enum

from map_generator.base_map_gen import BaseMapGenerator
from map_generator.factory import MapGeneratorFactory
from map_generator.rosnav.wrapper import create_indoor_map, create_outdoor_map


class MAP_TYPE(Enum):
    indoor = "indoor"
    outdoor = "outdoor"


@MapGeneratorFactory.register("rosnav")
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
        **kwargs
    ):
        super().__init__(height, width)

        self.map_type = MAP_TYPE(map_type.lower())

        # indoor parameters
        self.iterations = iterations
        self.cr = corridor_radius

        # outdoor parameters
        self.obstacle_num = obstacle_num
        # extra radius for obstacles, middle point is the obstacle center
        self.obstacle_extra_radius = obstacle_extra_radius

    def generate_grid_map(self) -> np.ndarray:
        return (
            create_indoor_map(
                height=self.height,
                width=self.width,
                corridor_radius=self.cr,
                iterations=self.iterations,
            )
            if self.map_type == MAP_TYPE.indoor
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
