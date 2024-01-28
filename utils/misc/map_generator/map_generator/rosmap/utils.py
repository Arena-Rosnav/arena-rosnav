import numpy as np
from ..constants import ROSNAV_PLACEMENT_TRIES


def initialize_map(height: int, width: int, type="bordered") -> np.ndarray:
    # create empty map with format given by height,width and initialize empty tree
    grid_map = np.tile(1, [height, width])
    if type == "bordered":
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

        if check_if_free(grid_map, map_default,
                         random_x - corridor_radius, random_x + corridor_radius, random_y - corridor_radius, random_y + corridor_radius):
            return [random_x, random_y]

    return []


def check_if_free(grid: np.ndarray, free_val: int, start_x: int, end_x: int, start_y: int, end_y: int) -> bool:
    sub_grid = grid[(start_x):(end_x + 1), (start_y):(end_y + 1)]
    if np.all(sub_grid == free_val):
        return True
    return False


def rotate_scale_point(point: np.ndarray, angle: float, center: np.ndarray, scale: float):
    """
    Rotate a point around a given center.
    Params:
        point: the point to be rotated (array of length 2)
        angle: amount to rotate in radians
        center: the point around which the rotation should be performed (array of length 2)
        scale: factor to scale the position after calculation
    """
    x, y = point
    cx, cy = center

    # Translate the point to the origin
    translated_x = x - cx
    translated_y = y - cy

    # Perform the rotation
    rotated_x = translated_x * \
        np.cos(angle) - translated_y * np.sin(angle)
    rotated_y = translated_x * \
        np.sin(angle) + translated_y * np.cos(angle)

    # Translate the point back to its original position
    new_x = rotated_x + cx
    new_y = rotated_y + cy

    # scale the position
    new_x *= scale
    new_y *= scale

    return new_x, new_y
