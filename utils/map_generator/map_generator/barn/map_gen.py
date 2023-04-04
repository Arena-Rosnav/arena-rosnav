import numpy as np

from map_generator.base_map_gen import BaseMapGenerator
from map_generator.factory import MapGeneratorFactory
from map_generator.utils.general import calc_infl_rad_cells
from map_generator.barn.obstacle_map import ObstacleMap
from map_generator.barn.robot_map import RobotMap


@MapGeneratorFactory.register("barn")
class BarnMapGenerator(BaseMapGenerator):
    def __init__(
        self,
        height: int,
        width: int,
        robot_infl_radius: float,
        map_resolution: float,
        smooth_iter: int = 5,
        fill_pct: float = 0.25,
        seed: int = None,
        *args,
        **kwargs
    ):
        super().__init__(height, width - 2)  # - 2 for side walls which are added later
        self.smooth_iter = smooth_iter
        self.fill_pct = fill_pct
        self.seed = seed

        self.infl_radius_cells = calc_infl_rad_cells(robot_infl_radius, map_resolution)
        self.robot_radius_extra_cells = self.infl_radius_cells - 1

    def generate_grid_map(self) -> np.ndarray:
        # sourcery skip: assign-if-exp, reintroduce-else, swap-if-expression
        obstacle_map_obj = ObstacleMap(
            rows=self.height,
            cols=self.width,
            rand_fill_pct=self.fill_pct,
            smooth_iter=self.smooth_iter,
            seed=self.seed,
        )
        obstacle_map = obstacle_map_obj.generate_map()

        if not BarnMapGenerator.check_for_paths(
            obstacle_map, self.robot_radius_extra_cells, self.infl_radius_cells
        ):
            return self.generate_grid_map()

        # add side walls as the map was initially open on both sides for barn
        np_obs_map = np.array(obstacle_map)
        ones_col = np.ones((np_obs_map.shape[0], 1))
        return np.concatenate((ones_col, np_obs_map, ones_col), axis=1)

    @staticmethod
    def check_for_paths(
        obstacle_map: np.ndarray, robot_radius_extra_cells: int, infl_radius_cells: int
    ) -> bool:
        robot_map = RobotMap(obstacle_map, robot_radius_extra_cells, infl_radius_cells)

        start_region = robot_map.biggest_left_region()
        end_region = robot_map.biggest_right_region()

        return robot_map.regions_connected(start_region, end_region)


def test():
    map_gen = BarnMapGenerator(
        height=70, width=50, robot_infl_radius=0.3, map_resolution=0.25
    )
    grid_map = map_gen.generate_grid_map()
    None


if __name__ == "__main__":
    test()
