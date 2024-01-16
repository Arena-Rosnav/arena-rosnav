import numpy as np
from ..constants import ROSNAV_PLACEMENT_TRIES


def initialize_map(height: int, width: int, type="indoor") -> np.ndarray:
    # create empty map with format given by height,width and initialize empty tree
    grid_map = np.tile(1, [height, width])
    if type == "outdoor":
        grid_map[slice(1, height - 1), slice(1, width - 1)] = 0
    return grid_map


def sample(grid_map: np.ndarray, corridor_radius: int, map_default: int) -> list:
    # sample position from map within boundary and leave tolerance for corridor width
    # prevent overlapping obstacles
    for _ in range(ROSNAV_PLACEMENT_TRIES):
        random_x = np.random.choice(
            range(corridor_radius + 2, grid_map.shape[0] - corridor_radius - 1)
        )
        random_y = np.random.choice(
            range(corridor_radius + 2, grid_map.shape[1] - corridor_radius - 1)
        )

        sub_grid = grid_map[(random_x - corridor_radius):(random_x + corridor_radius + 1),
                            (random_y - corridor_radius):(random_y + corridor_radius + 1)]
        if np.all(sub_grid == map_default):
            return [random_x, random_y]

    return []


def rotate_point(point, angle, center):
    """Rotate a point around a given center."""
    angle_rad = np.radians(angle)
    x, y = point
    cx, cy = center

    # Translate the point to the origin
    translated_x = x - cx
    translated_y = y - cy

    # Perform the rotation
    rotated_x = translated_x * np.cos(angle_rad) - translated_y * np.sin(angle_rad)
    rotated_y = translated_x * np.sin(angle_rad) + translated_y * np.cos(angle_rad)

    # Translate the point back to its original position
    new_x = rotated_x + cx
    new_y = rotated_y + cy

    return new_x, new_y
