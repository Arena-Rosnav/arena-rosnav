import numpy as np


def get_constellation(node1, node2):
    # there are two relevant constellations for the 2 nodes, which must be considered when creating the horizontal and vertical path
    # 1: lower left and upper right
    # 2: upper left and lower right
    # each of the 2 constellation have 2 permutations which must be considered as well
    constellation1 = {
        "permutation1": node1[0] > node2[0] and node1[1] < node2[1],  # x1>x2 and y1<y2
        "permutation2": node1[0] < node2[0] and node1[1] > node2[1],
    }  # x1<x2 and y1>y2
    if constellation1["permutation1"] or constellation1["permutation2"]:
        return 1
    else:
        return 2


def create_path(node1, node2, corridor_radius, map):
    coin_flip = np.random.random()
    x1, x2 = sorted(
        [node1[0], node2[0]]
    )  # x and y coordinates must be sorted for usage with range function
    y1, y2 = sorted([node1[1], node2[1]])
    if get_constellation(node1, node2) == 1:  # check which constellation
        if coin_flip >= 0.5:
            # randomly determine the curvature of the path (right turn/left turn)
            map[
                slice(x1 - corridor_radius, x1 + corridor_radius + 1),
                range(y1 - corridor_radius, y2 + 1 + corridor_radius, 1),
            ] = 0  # horizontal path
            map[
                range(x1 - corridor_radius, x2 + 1 + corridor_radius, 1),
                slice(y1 - corridor_radius, y1 + corridor_radius + 1),
            ] = 0  # vertical path
        else:
            map[
                slice(x2 - corridor_radius, x2 + corridor_radius + 1),
                range(y1 - corridor_radius, y2 + 1 + corridor_radius, 1),
            ] = 0  # horizontal path
            map[
                range(x1 - corridor_radius, x2 + 1 + corridor_radius, 1),
                slice(y2 - corridor_radius, y2 + corridor_radius + 1),
            ] = 0  # vertical path
    else:
        if coin_flip >= 0.5:
            # randomly determine the curvature of the path (right turn/left turn)
            map[
                slice(x1 - corridor_radius, x1 + corridor_radius + 1),
                range(y1 - corridor_radius, y2 + 1 + corridor_radius, 1),
            ] = 0  # horizontal path
            map[
                range(x1 - corridor_radius, x2 + 1 + corridor_radius, 1),
                slice(y2 - corridor_radius, y2 + corridor_radius + 1),
            ] = 0  # vertical path
        else:
            map[
                slice(x2 - corridor_radius, x2 + corridor_radius + 1),
                range(y1 - corridor_radius, y2 + 1 + corridor_radius, 1),
            ] = 0  # horizontal path
            map[
                range(x1 - corridor_radius, x2 + 1 + corridor_radius, 1),
                slice(y1 - corridor_radius, y1 + corridor_radius + 1),
            ] = 0  # vertical path
