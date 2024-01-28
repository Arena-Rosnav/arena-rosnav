import numpy as np

from map_generator.msg import Obstacle
from geometry_msgs.msg import Pose, Point, Quaternion

from ..utils import *


def create_outdoor_map(
    height: int, width: int, obstacle_number: int, obstacle_extra_radius: int, map_resolution: float
) -> (np.ndarray, dict):
    grid_map = initialize_map(height, width, type="outdoor")
    obstacles = []
    obstacle_grid = np.tile(0, [height, width])
    for _ in range(obstacle_number):
        random_position = sample(obstacle_grid, obstacle_extra_radius, 0)
        if random_position == []:  # couldn't find free spot
            continue

        obst = Obstacle(model_name="tree", position=Pose(position=Point(
            x=random_position[1], y=height-random_position[0], z=0), orientation=Quaternion()))

        # scale position with map resolution
        obst.position.position.x *= map_resolution
        obst.position.position.y *= map_resolution

        obstacles.append(obst)

        obstacle_grid[
            slice(
                random_position[0] - obstacle_extra_radius,
                random_position[0] + obstacle_extra_radius + 1,
            ),  # create 1 pixel obstacles with extra radius if specified
            slice(
                random_position[1] - obstacle_extra_radius,
                random_position[1] + obstacle_extra_radius + 1,
            ),
        ] = 1

    obstacle_data = {"obstacles": obstacles, "occupancy": obstacle_grid}
    return grid_map, obstacle_data