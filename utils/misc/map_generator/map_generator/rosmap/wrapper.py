import numpy as np

from map_generator.msg import Obstacle
from geometry_msgs.msg import Pose, Point, Quaternion

from .tree import *


def create_outdoor_map(
    height: int, width: int, obstacle_number: int, obstacle_extra_radius: int
) -> (np.ndarray, dict):
    grid_map = initialize_map(height, width, type="outdoor")
    obstacles = []
    obstacle_grid = np.tile(0, [height, width])
    for _ in range(obstacle_number):
        random_position = sample(obstacle_grid, obstacle_extra_radius, 0)
        if random_position == []:  # couldn't find free spot
            continue

        obstacles.append(Obstacle(model_name="tree", position=Pose(position=Point(
            x=random_position[1], y=height-random_position[0], z=0), orientation=Quaternion())))

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


def create_canteen_map(
    height: int, width: int, obstacle_number: int, obstacle_extra_radius: int, chair_chance: float
) -> (np.ndarray, dict):
    grid_map = initialize_map(height, width, type="outdoor")
    obstacles = []
    obstacle_grid = np.tile(0, [height, width])
    for _ in range(obstacle_number):
        random_position = sample(obstacle_grid, obstacle_extra_radius, 0)
        if random_position == []:  # couldn't find free spot
            continue

        rot = np.random.random() * np.pi
        rot = np.pi/2 if np.random.random() > 0.5 else 0
        table_pos = [random_position[1], height-random_position[0]]

        # place a table and random amount of chairs around it
        obstacles.append(Obstacle(model_name="table", position=Pose(position=Point(
            x=table_pos[0], y=table_pos[1], z=0), orientation=Quaternion(z=rot))))

        # each chair has a chance to spawn
        # obstacles += place_rect(table_pos, rot, chair_chance) 
        obstacles += place_round(table_pos, rot, chair_chance)

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


def place_rect(center, rot: float, chair_chance: float):
    """
    Place the chairs around a rectangular table
    """
    chairs = []

    # each chair has a chance to spawn
    if np.random.random() <= chair_chance:
        pos = [center[0] - 1, center[1] - 0.8]
        pos = rotate_point(pos, np.rad2deg(rot), center)
        chairs.append(Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=rot))))
    if np.random.random() <= chair_chance:
        pos = [center[0] - 1, center[1] + 0.8]
        pos = rotate_point(pos, np.rad2deg(rot), center)
        chairs.append(Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=rot))))
    if np.random.random() <= chair_chance:
        pos = [center[0] + 1, center[1] - 0.8]
        pos = rotate_point(pos, np.rad2deg(rot), center)
        chairs.append(Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=np.pi + rot))))
    if np.random.random() <= chair_chance:
        pos = [center[0] + 1, center[1] + 0.8]
        pos = rotate_point(pos, np.rad2deg(rot), center)
        chairs.append(Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=np.pi + rot))))

    return chairs


def place_round(center, rot: float, chair_chance: float):
    """
    Place the chairs around a round table
    """
    chairs = []

    # each chair has a chance to spawn
    if np.random.random() <= chair_chance:
        pos = [center[0] - 1, center[1]]
        pos = rotate_point(pos, np.rad2deg(rot), center)
        chairs.append(Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=rot))))
    if np.random.random() <= chair_chance:
        pos = [center[0], center[1] + 1]
        pos = rotate_point(pos, np.rad2deg(rot), center)
        chairs.append(Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=rot + np.pi/4))))
    if np.random.random() <= chair_chance:
        pos = [center[0] + 1, center[1]]
        pos = rotate_point(pos, np.rad2deg(rot), center)
        chairs.append(Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=rot + np.pi/2))))
    if np.random.random() <= chair_chance:
        pos = [center[0], center[1] - 1]
        pos = rotate_point(pos, np.rad2deg(rot), center)
        chairs.append(Obstacle(model_name="chair", position=Pose(position=Point(
            x=pos[0], y=pos[1], z=0), orientation=Quaternion(z=rot + 1.5 * np.pi))))

    return chairs
