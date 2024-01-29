import numpy as np
import random

from map_generator.msg import Obstacle
from geometry_msgs.msg import Pose, Point, Quaternion

from ..utils import *


def create_warehouse_map(
    height: int,
    width: int,
    pallet_jacks: int,
    hor_dist_shelfs_min: int,
    hor_dist_shelfs_max: int,
    vert_dist_shelfs_min: int,
    vert_dist_shelfs_max: int,
    map_resolution: float
) -> (np.ndarray, dict):
    grid_map = initialize_map(height, width, type="bordered")
    obstacles = []
    obstacle_grid = np.tile(0, [height, width])

    # RANDOM PARAMS
    print(pallet_jacks)

    # random between horizontal and vertical
    rot = np.pi / 2 if np.random.random() < 0.5 else 0

    # how far the shelf right next to each other should be
    # replace /2 with * map_resolution?
    vert_dist = np.random.randint(
        vert_dist_shelfs_min, vert_dist_shelfs_max) / 2
    # how far the next shelf row should be
    hor_dist = np.random.randint(hor_dist_shelfs_min, hor_dist_shelfs_max) / 2

    # start pos
    start_vert = np.random.randint(int(vert_dist/2)+1, int(vert_dist * 2))
    start_hor = np.random.randint(int(hor_dist/2)+1, int(hor_dist * 2))

    if not rot == 0:
        start_hor, start_vert = start_vert, start_hor
        vert_dist, hor_dist = hor_dist, vert_dist

    # how many shelf fit in one row
    max_cols = int((width - start_hor + hor_dist / 2) / hor_dist)
    max_cols = max(np.random.randint(int(max_cols / 2), max_cols + 1), 1)
    # how many rows fit in map
    max_rows = int((height - start_vert + vert_dist / 2) / vert_dist)
    max_rows = max(np.random.randint(int(max_rows / 2), max_rows + 1), 1)

    for row in range(max_rows):
        for col in range(max_cols):
            point = Point(
                x=start_hor + hor_dist * col,
                y=start_vert + vert_dist * row,
                z=0
            )
            obst = Obstacle(model_name="warehouse_shelf", position=Pose(
                position=point,
                orientation=Quaternion(z=rot + np.pi / 2)
            ))

            obst.position.position.x *= map_resolution
            obst.position.position.y *= map_resolution

            obstacles.append(obst)

            # Block space in grid
            # TODO Rework, currently not working properly
            obstacle_grid[
                slice(int(point.y+1),
                      int(point.y + 3)),
                slice(int(point.x),
                      int(point.x + hor_dist + 1)),
            ] = 1

    for _ in range(pallet_jacks):
        random_position = sample(obstacle_grid, 3, 0)
        if random_position == []:  # couldn't find free spot
            continue

        obst_rot = np.random.random() * 2 * np.pi

        # Spawn a pallet jack
        options = ["pallet_jack_1", "pallet_jack_2", "pallet_jack_3"]
        jack = Obstacle(model_name=random.choice(options), position=Pose(position=Point(
            x=random_position[1], y=height-random_position[0], z=0), orientation=Quaternion(z=obst_rot)))

        jack.position.position.x *= map_resolution
        jack.position.position.y *= map_resolution

        obstacles.append(jack)

    obstacle_data = {"obstacles": obstacles, "occupancy": obstacle_grid}
    return grid_map, obstacle_data
