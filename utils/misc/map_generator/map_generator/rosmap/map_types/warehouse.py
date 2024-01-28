import numpy as np

from map_generator.msg import Obstacle
from geometry_msgs.msg import Pose, Point, Quaternion

from ..utils import *


def create_warehouse_map(
    height: int,
    width: int,
    map_resolution: float
) -> (np.ndarray, dict):
    grid_map = initialize_map(height, width, type="bordered")
    obstacles = []
    obstacle_grid = np.tile(0, [height, width]) # leave out for now

    # random between horizontal and vertical
    rot = np.pi / 2 if np.random.random() < 0.5 else 0

    for _ in range(1000):
        random_position = sample(obstacle_grid, 0, 0)
        if random_position == []:  # couldn't find free spot
            continue

        obst = Obstacle(model_name="tree", position=Pose(position=Point(
            x=random_position[1], y=height-random_position[0], z=0), orientation=Quaternion()))

        # scale position with map resolution
        obst.position.position.x *= map_resolution
        obst.position.position.y *= map_resolution

        obstacles.append(obst)

    obstacle_data = {"obstacles": obstacles, "occupancy": obstacle_grid}
    return grid_map, obstacle_data
