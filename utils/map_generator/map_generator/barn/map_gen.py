from typing import Tuple

import numpy as np
import rospy

from map_generator.base_map_gen import BaseMapGenerator
from map_generator.factory import MapGeneratorFactory
from map_generator.utils.general import calc_infl_rad_cells
from map_generator.barn.obstacle_map import ObstacleMap
from map_generator.barn.robot_map import RobotMap
from map_generator.constants import (
    BARN_MAX_RECURSION_DEPTH,
    MapGenerators,
    MAP_GENERATOR_NS,
)


@MapGeneratorFactory.register(MapGenerators.BARN)
class BarnMapGenerator(BaseMapGenerator):
    """
    BarnMapGenerator is a class that generates a grid map with the barn algorithm for the Rosnav environment.

    Args:
        height (int): The height of the map.
        width (int): The width of the map.
        robot_infl_radius (float): The inflation radius of the robot.
        map_resolution (float): The resolution of the map.
        smooth_iter (int, optional): The number of smoothing iterations. Defaults to 5.
        fill_pct (float, optional): The percentage of the map to be filled with obstacles. Defaults to 0.25.
        seed (int, optional): The seed for random number generation. Defaults to None.

    Attributes:
        smooth_iter (int): The number of smoothing iterations.
        fill_pct (float): The percentage of the map to be filled with obstacles.
        seed (int): The seed for random number generation.
        map_resolution (float): The resolution of the map.
        robot_infl_radius (float): The inflation radius of the robot.
        infl_radius_cells (int): The inflation radius in cells.
        robot_radius_extra_cells (int): The extra cells added to the robot radius.

    Methods:
        update_params: Update the parameters of the map generator.
        retrieve_params: Retrieve the parameters of the map generator.
        generate_grid_map: Generate a grid map for the barn environment.
        check_for_paths: Check if there are paths connecting the start and end regions.
    """

    def __init__(
        self,
        height: int,
        width: int,
        robot_infl_radius: float,
        map_resolution: float,
        smooth_iter: int = 5,
        fill_pct: float = 0.2,
        seed: int = None,
        *args,
        **kwargs
    ):
        """
        Initialize the MapGenerator object.

        Args:
            height (int): The height of the map in cells.
            width (int): The width of the map in cells.
            robot_infl_radius (float): The radius of the robot's influence in meters.
            map_resolution (float): The resolution of the map in meters per cell.
            smooth_iter (int, optional): The number of smoothing iterations to perform. Defaults to 5.
            fill_pct (float, optional): The percentage of the map to be filled with obstacles. Defaults to 0.25.
            seed (int, optional): The seed for the random number generator. Defaults to None.
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        super().__init__(
            height, width - 2, map_resolution
        )  # - 2 for side walls which are added later
        self.smooth_iter = smooth_iter
        self.fill_pct = fill_pct
        self.seed = seed

        self.map_resolution = map_resolution
        self.robot_infl_radius = robot_infl_radius

        self.infl_radius_cells = calc_infl_rad_cells(robot_infl_radius, map_resolution)
        self.robot_radius_extra_cells = self.infl_radius_cells - 1

    def update_params(
        self, height: int, width: int, fill_pct: float, smooth_iter: int, map_res: float
    ):
        """
        Update the parameters of the map generator.

        Args:
            height (int): The new height of the map.
            width (int): The new width of the map.
            fill_pct (float): The new percentage of the map to be filled with obstacles.
            smooth_iter (int): The new number of smoothing iterations.
            map_res (float): The new resolution of the map.
        """
        if map_res != self.map_resolution:
            # calculate inflation radius in cells when map resolution changes
            self.infl_radius_cells = calc_infl_rad_cells(
                self.robot_infl_radius, map_res
            )
            self.robot_radius_extra_cells = self.infl_radius_cells - 1
        super().update_params(height, width - 2, map_res)
        self.fill_pct, self.smooth_iter = fill_pct, smooth_iter

    def retrieve_params(self) -> Tuple[int, int, float, int, float]:
        """
        Retrieve the parameters of the map generator.

        Returns:
            Tuple[int, int, float, int, float]: A tuple containing the height, width, fill percentage,
            number of smoothing iterations, and map resolution.
        """
        height, width, map_res = super().retrieve_params()
        fill_pct = rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/fill_pct"), self.fill_pct
        )
        smooth_iter = rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/smooth_iter"), self.smooth_iter
        )

        return height, width, fill_pct, smooth_iter, map_res

    def generate_grid_map(self, call_depth: int = 0) -> np.ndarray:
        """
        Generate a grid map for the barn environment.

        Args:
            call_depth (int, optional): The recursion depth. Defaults to 0.

        Returns:
            np.ndarray: The generated grid map.
        """
        if call_depth > BARN_MAX_RECURSION_DEPTH:
            raise RecursionError(
                "[Barn] Recursion depth exceeded, please check your parameters!"
            )
        super().generate_grid_map()

        # sourcery skip: assign-if-exp, reintroduce-else, swap-if-expression
        obstacle_map_obj = ObstacleMap(
            rows=self.height,
            cols=self.width,
            rand_fill_pct=self.fill_pct,
            smooth_iter=self.smooth_iter,
            seed=self.seed,
        )
        obstacle_map = obstacle_map_obj.generate_map()

        try:
            if not BarnMapGenerator.check_for_paths(
                obstacle_map, self.robot_radius_extra_cells, self.infl_radius_cells
            ):
                rospy.loginfo("[Barn] No path found, regenerating map!")
                return self.generate_grid_map(call_depth + 1)
        except IndexError:
            rospy.loginfo(
                "[Barn] Index out of bound during 'check_for_paths', regenerating map!"
            )
            return self.generate_grid_map(call_depth + 1)

        # add side walls as the map was initially open on both sides for barn
        np_obs_map = np.array(obstacle_map)
        ones_col = np.ones((np_obs_map.shape[0], 1))
        return np.concatenate((ones_col, np_obs_map, ones_col), axis=1)

    @staticmethod
    def check_for_paths(
        obstacle_map: np.ndarray, robot_radius_extra_cells: int, infl_radius_cells: int
    ) -> bool:
        """
        Check if there are paths connecting the start and end regions.

        Args:
            obstacle_map (np.ndarray): The obstacle map.
            robot_radius_extra_cells (int): The extra cells added to the robot radius.
            infl_radius_cells (int): The inflation radius in cells.

        Returns:
            bool: True if there are paths connecting the start and end regions, False otherwise.
        """
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
