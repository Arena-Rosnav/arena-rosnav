import numpy as np

from map_generator.msg import Obstacle
from geometry_msgs.msg import Pose, Point, Quaternion

from ..utils import *

def create_canteen_map(
    height: int, width: int, obstacle_number: int, obstacle_extra_radius: int, chair_chance: float, map_resolution: float
) -> (np.ndarray, dict):
    grid_map = initialize_map(height, width, type="bordered")
    obstacles = []
    obstacle_grid = np.tile(0, [height, width])
    for _ in range(obstacle_number):
        random_position = sample(obstacle_grid, obstacle_extra_radius, 0)
        if random_position == []:  # couldn't find free spot
            continue

        rot = np.random.random() * np.pi
        # rot = np.pi/2 if np.random.random() > 0.5 else 0
        table_pos = [random_position[1], height-random_position[0]]

        # place a table and random amount of chairs around it
        table = Obstacle(model_name="table", position=Pose(position=Point(
            x=table_pos[0], y=table_pos[1], z=0), orientation=Quaternion(z=rot)))
        # scale position with map resolution
        table.position.position.x *= map_resolution
        table.position.position.y *= map_resolution
        obstacles.append(table)

        # each chair has a chance to spawn
        # obstacles += place_rect(table_pos, rot, chair_chance)
        obstacles += place_round(table_pos, rot, chair_chance, map_resolution)

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


def place_rect(center: np.ndarray, rot: float, chair_chance: float, map_resolution: float):
    """
    Place the chairs around a rectangular table
    """
    chairs = []

    def chair(init_pos: np.ndarray, rot: float):
        pos = rotate_scale_point(init_pos, rot, center, map_resolution)
        return Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=rot)))

    # each chair has a chance to spawn
    if np.random.random() <= chair_chance:
        chairs.append(chair([center[0] - 1, center[1] - 0.8], rot))
    if np.random.random() <= chair_chance:
        chairs.append(chair([center[0] - 1, center[1] + 0.8], rot))
    if np.random.random() <= chair_chance:
        chairs.append(chair(
            [center[0] + 1, center[1] - 0.8], rot + np.pi))
    if np.random.random() <= chair_chance:
        chairs.append(chair(
            [center[0] + 1, center[1] + 0.8], rot + np.pi))

    return chairs


def place_round(center, rot: float, chair_chance: float, map_resolution: float):
    """
    Place the chairs around a round table
    """
    chairs = []

    def chair(init_pos: np.ndarray, rot: float):
        pos = rotate_scale_point(init_pos, rot, center, map_resolution)
        return Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=rot)))

    # each chair has a chance to spawn
    if np.random.random() <= chair_chance:
        chairs.append(chair([center[0] - 1, center[1]], rot))
    if np.random.random() <= chair_chance:
        chairs.append(chair([center[0], center[1] - 1], rot + 0.5 * np.pi))
    if np.random.random() <= chair_chance:
        chairs.append(chair([center[0] + 1, center[1]], rot + 1 * np.pi))
    if np.random.random() <= chair_chance:
        chairs.append(chair([center[0], center[1] + 1], rot + 1.5 * np.pi))

    return chairs
