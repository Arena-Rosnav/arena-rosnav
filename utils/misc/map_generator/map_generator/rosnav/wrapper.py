import numpy as np

from .path import create_path
from .tree import *


def create_random_map(
    height: int,
    width: int,
    corridor_radius: int,
    iterations: int,
    obstacle_number: int,
    obstacle_extra_radius: int,
):
    return (
        create_indoor_map(height, width, corridor_radius, iterations)
        if np.random.random() >= 0.5
        else create_outdoor_map(height, width, obstacle_number, obstacle_extra_radius)
    )


def create_indoor_map(
    height: int, width: int, corridor_radius: int, iterations: int
) -> np.ndarray:
    tree = []  # initialize empty tree
    grid_map = initialize_map(height, width)
    insert_root_node(grid_map, tree)
    for _ in range(iterations):  # create as many paths/nodes as defined in iteration
        random_position = sample(grid_map, corridor_radius)
        nearest_node = find_nearest_node(
            random_position, tree
        )  # nearest node must be found before inserting the new node into the tree, else nearest node will be itself
        insert_new_node(random_position, tree, grid_map)
        create_path(random_position, nearest_node, corridor_radius, grid_map)
    return grid_map


def create_outdoor_map(
    height: int, width: int, obstacle_number: int, obstacle_extra_radius: int
) -> np.ndarray:
    grid_map = initialize_map(height, width, type="outdoor")
    for _ in range(obstacle_number):
        random_position = sample(grid_map, obstacle_extra_radius)
        grid_map[
            slice(
                random_position[0] - obstacle_extra_radius,
                random_position[0] + obstacle_extra_radius + 1,
            ),  # create 1 pixel obstacles with extra radius if specified
            slice(
                random_position[1] - obstacle_extra_radius,
                random_position[1] + obstacle_extra_radius + 1,
            ),
        ] = 1
    return grid_map
