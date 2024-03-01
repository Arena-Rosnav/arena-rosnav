import numpy as np
import random

from map_generator.msg import Obstacle
from geometry_msgs.msg import Pose, Point, Quaternion

from ..utils import *


def create_office_map(
    height: int,
    width: int,
    hor_dist_table_min: int,
    hor_dist_table_max: int,
    vert_dist_table_min: int,
    vert_dist_table_max: int,
    row_chance: float,
    map_resolution: float
) -> (np.ndarray, dict):
    grid_map = initialize_map(height, width, type="bordered")
    obstacles = []
    obstacle_grid = np.tile(0, [height, width])

    # RANDOM PARAMS

    # random between horizontal and vertical
    rot = np.pi / 2 if np.random.random() < 0.5 else 0

    # how far the shelf right next to each other should be
    # replace /2 with * map_resolution?
    vert_dist = np.random.randint(
        vert_dist_table_min, vert_dist_table_max) / 2
    # how far the next shelf row should be
    hor_dist = np.random.randint(hor_dist_table_min, hor_dist_table_max) / 2

    # start pos
    start_vert = np.random.randint(int(vert_dist/2)+1, int(vert_dist * 2))
    start_hor = np.random.randint(int(hor_dist/2)+1, int(hor_dist * 2))

    if not rot == 0:
        start_hor, start_vert = start_vert, start_hor
        vert_dist, hor_dist = hor_dist, vert_dist

    # how many shelf fit in one row
    max_cols = int((width - start_hor + hor_dist / 2) / hor_dist)
    # how many rows fit in map
    max_rows = int((height - start_vert + vert_dist / 2) / vert_dist)

    for row in range(max_rows):

        # skip certain number of rows depending on row_chance
        if np.random.random() > row_chance:
            continue

        start_idx = np.random.randint(0, int(max_cols/2))
        end_idx = np.random.randint(0, int(max_cols/2))

        for col in range(start_idx, max_cols - end_idx):
            point = Point(
                x=start_hor + hor_dist * col,
                y=start_vert + vert_dist * row,
                z=0
            )
            if rot == 0:
                desk_point = Point(x=point.x, y=point.y + 2, z=0)
                chair_point_1 = Point(x=point.x, y=point.y - 1, z=0)
                chair_point_2 = Point(x=point.x, y=point.y + 3, z=0)
            else:
                desk_point = Point(x=point.x + 2, y=point.y, z=0)
                chair_point_1 = Point(x=point.x + 3, y=point.y, z=0)
                chair_point_2 = Point(x=point.x - 1, y=point.y, z=0)

            # DESK 1
            desk_1 = Obstacle(model_name="office_desk", position=Pose(
                position=point,
                orientation=Quaternion(z=rot + (0 if rot == 0 else np.pi))
            ))

            obstacles.append(desk_1)

            # DESK 2
            desk_2 = Obstacle(model_name="office_desk", position=Pose(
                position=desk_point,
                orientation=Quaternion(z=rot + (np.pi if rot == 0 else 0))
            ))
            obstacles.append(desk_2)

            # CHAIR 1
            chair_1 = Obstacle(model_name="office_chair", position=Pose(
                position=chair_point_1,
                orientation=Quaternion(z=rot + 1.5 * np.pi)
            ))
            obstacles.append(chair_1)

            # CHAIR 2
            chair_2 = Obstacle(model_name="office_chair", position=Pose(
                position=chair_point_2,
                orientation=Quaternion(z=rot + 0.5 * np.pi)
            ))
            obstacles.append(chair_2)

    # scale all obstacles by map_resolution
    for obst in obstacles:
        obst.position.position.x *= map_resolution
        obst.position.position.y *= map_resolution

    obstacle_data = {"obstacles": obstacles, "occupancy": obstacle_grid}
    return grid_map, obstacle_data
