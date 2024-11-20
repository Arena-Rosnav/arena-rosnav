import math
from typing import List, Tuple


class Node:
    """
    Represents a node in the A* algorithm.

    Attributes:
        parent (list): The parent node of the current node.
        r (int): The row coordinate of the node.
        c (int): The column coordinate of the node.
        g (int): The cost of the path from the start node to the current node.
        h (int): The estimated cost from the current node to the goal node.
        f (int): The sum of g and h, representing the total cost of the node.
    """

    def __init__(self, parent: list, coord: list):
        self.parent = parent
        self.r = coord[0]
        self.c = coord[1]

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        """
        Check if two objects are equal based on their row and column values.

        Args:
            other: The object to compare with.

        Returns:
            True if the objects are equal, False otherwise.
        """
        return self.r == other.r and self.c == other.c


class AStarSearch:
    """
    A* search algorithm implementation for pathfinding on a grid map.

    Args:
        map (List[List[int]]): The grid map represented as a 2D list of integers.
        infl_rad_cells (int): The inflation radius in cells.

    Attributes:
        map (List[List[int]]): The grid map represented as a 2D list of integers.
        map_rows (int): The number of rows in the grid map.
        map_cols (int): The number of columns in the grid map.
        infl_rad_cells (int): The inflation radius in cells.

    Methods:
        __call__(start_coord, end_coord, dist_map): Performs the A* search algorithm to find the path from start_coord to end_coord.
        return_path(end_node): Generates the path from the end node to the start node.
    """

    # infl_rad_cells: the inflation radius, in cells
    def __init__(self, map: List[List[int]], infl_rad_cells: int):
        self.map = map
        self.map_rows = len(map)
        self.map_cols = len(map[0])
        self.infl_rad_cells = infl_rad_cells

    # dist_map: grid with the distances to closest obstacle at each point
    def __call__(
        self,
        start_coord: List[Tuple[int, int]],
        end_coord: List[Tuple[int, int]],
        dist_map: List[List[int]],
    ):
        """
        Performs the A* search algorithm to find the path from start_coord to end_coord.

        Args:
            start_coord (List[Tuple[int, int]]): The coordinates of the start position.
            end_coord (List[Tuple[int, int]]): The coordinates of the end position.
            dist_map (List[List[int]]): The grid with the distances to the closest obstacle at each point.

        Returns:
            List[Tuple[int, int]]: The path from the start position to the end position.
        """
        # limit turns to 45 degrees
        valid_moves_dict = {
            (0, 1): [(-1, 1), (0, 1), (1, 1)],
            (1, 1): [(0, 1), (1, 1), (1, 0)],
            (1, 0): [(1, 1), (1, 0), (1, -1)],
            (1, -1): [(1, 0), (1, -1), (0, -1)],
            (0, -1): [(1, -1), (0, -1), (-1, -1)],
            (-1, -1): [(0, -1), (-1, -1), (-1, 0)],
            (-1, 0): [(-1, -1), (-1, 0), (-1, 1)],
            (-1, 1): [(-1, 0), (-1, 1), (0, 1)],
        }

        # cost factor for cells within the inflation radius
        penalty_factor = 5.0

        # initialize start and end nodes
        start_node = Node(None, start_coord)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end_coord)
        end_node.g = end_node.h = end_node.f = 0

        # initialize lists to track nodes we've visited or not
        visited = []
        not_visited = []

        # add start to nodes yet to be processed
        not_visited.append(start_node)

        # while there are nodes to process
        while not_visited:
            # get lowest cost next node
            curr_node = not_visited[0]
            curr_idx = 0
            for idx, node in enumerate(not_visited):
                if node.f < curr_node.f:
                    curr_node = node
                    curr_idx = idx

            # pop this node from the unvisited list and add to visited list
            not_visited.pop(curr_idx)
            visited.append(curr_node)

            # if this node is at end of the path, return
            if curr_node == end_node:
                return self.return_path(curr_node)

            # get all valid moves (either straight or 45 degree turn)
            valid_moves = []
            if curr_node == start_node:
                # if start node, can go any direction
                valid_moves = [
                    (0, 1),
                    (1, 1),
                    (1, 0),
                    (1, -1),
                    (0, -1),
                    (-1, -1),
                    (-1, 0),
                    (-1, 1),
                ]
            else:
                # otherwise, can only go straight or 45 degree turn
                moving_direction = (
                    curr_node.r - curr_node.parent.r,
                    curr_node.c - curr_node.parent.c,
                )
                valid_moves = valid_moves_dict.get(moving_direction)

            # find all valid, walkable neighbors of this node
            children = []
            for move in valid_moves:
                # calculate neighbor position
                child_pos = (curr_node.r + move[0], curr_node.c + move[1])

                # if outside the map, not possible
                if (
                    child_pos[0] < 0
                    or child_pos[0] >= self.map_rows
                    or child_pos[1] < 0
                    or child_pos[1] >= self.map_cols
                ):
                    continue

                # if a wall tile, not possible
                if self.map[child_pos[0]][child_pos[1]] == 1:
                    continue

                # also not possible to move between diagonal walls
                if (
                    move[0] != 0
                    and move[1] != 0
                    and self.map[curr_node.r + move[0]][curr_node.c] == 1
                    and self.map[curr_node.r][curr_node.c + move[1]] == 1
                ):
                    continue

                # if neighbor is possible to reach, add to list of neighbors
                child_node = Node(curr_node, child_pos)
                children.append(child_node)

            # loop through all walkable neighbors of this node
            for child in children:
                # if already processed, don't use
                if child in visited:
                    continue

                # calculate g value
                child_g = curr_node.g + math.sqrt(
                    (curr_node.r - child.r) ** 2 + (curr_node.c - child.c) ** 2
                )
                in_radius = dist_map[child.r][child.c] <= self.infl_rad_cells
                penalty = (
                    (penalty_factor / dist_map[child.r][child.c]) if in_radius else 0
                )

                # check if this node is already in the unprocessed list
                child_in_openset = False
                for node in not_visited:
                    if node == child:
                        child_in_openset = True

                        # if the node is already in the list, but with a higher cost,
                        # update the cost to this new lower one
                        if child_g < node.g:
                            node.parent = curr_node
                            node.g = child_g
                            node.h = math.sqrt(
                                ((child.r - end_node.r) ** 2)
                                + ((child.c - end_node.c) ** 2)
                            )

                            # distance from start + distance to end + factor to penalize cells close to walls
                            node.f = node.g + node.h + penalty

                # if child is not yet in the unprocessed list, add it
                if not child_in_openset:
                    child.g = child_g
                    child.h = math.sqrt(
                        ((child.r - end_node.r) ** 2) + ((child.c - end_node.c) ** 2)
                    )
                    child.f = child.g + child.f + penalty
                    not_visited.append(child)

    # generate the path from start to end
    def return_path(self, end_node: Node):
        """
        Generates the path from the end node to the start node.

        Args:
            end_node (Node): The end node of the path.

        Returns:
            List[Tuple[int, int]]: The path from the end position to the start position.
        """
        path = []
        curr_node = end_node
        while curr_node != None:
            path.append((curr_node.r, curr_node.c))
            curr_node = curr_node.parent

        path.reverse()
        return path
