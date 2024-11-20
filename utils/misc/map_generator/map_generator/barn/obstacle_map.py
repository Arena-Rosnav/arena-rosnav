from typing import List

import random


# class to generate occupancy grids using cellular automaton
class ObstacleMap:
    """
    Class representing an obstacle map.

    Attributes:
        rows (int): The number of rows in the map.
        cols (int): The number of columns in the map.
        rand_fill_pct (float): The initial fill percentage of the map.
        seed (int, optional): The random seed to use for generating the map. Defaults to None.
        smooth_iter (int, optional): The number of smoothing iterations to run. Defaults to 5.
        map (List[List[int]]): The obstacle map represented as a 2D list.

    Methods:
        generate_map(obs_map: List[List[int]] = None) -> List[List[int]]:
            Generates the obstacle map by filling it randomly and running smoothing iterations.
            If an existing map is provided, it will be used instead of generating a new one.

        get_map() -> List[List[int]]:
            Returns the obstacle map.

    Private Methods:
        _random_fill():
            Randomly fills the map using the initial fill percentage.
            The top and bottom rows are kept filled as walls.

        _smooth():
            Runs one smoothing iteration on the map.
            Tiles with more than 4 filled neighbors are filled, and tiles with less than 2 filled neighbors are emptied.

        _tile_neighbors(r: int, c: int) -> int:
            Returns the number of filled-in neighbors (8-neighborhood) for a given tile.

        _in_map(r: int, c: int) -> bool:
            Checks if the given coordinates are within the map boundaries.

    """

    def __init__(
        self,
        rows: int,
        cols: int,
        rand_fill_pct: float,
        seed: int = None,
        smooth_iter: int = 5,
    ):
        self.map = [[0 for _ in range(cols)] for _ in range(rows)]
        self.rows = rows
        self.cols = cols
        self.rand_fill_pct = rand_fill_pct
        self.seed = seed
        self.smooth_iter = smooth_iter

    def generate_map(self, obs_map: List[List[int]] = None) -> List[List[int]]:
        """
        Generates the obstacle map by filling it randomly and running smoothing iterations.
        If an existing map is provided, it will be used instead of generating a new one.

        Args:
            obs_map (List[List[int]], optional): An existing map to use instead of generating a new one. Defaults to None.

        Returns:
            List[List[int]]: The generated obstacle map.
        """
        if obs_map:
            assert len(obs_map) in [self.rows, self.cols]
            assert len(obs_map[0]) in [self.rows, self.cols]
            self.map = obs_map
        else:
            self._random_fill()

        for _ in range(self.smooth_iter):
            self._smooth()

        return self.map

    def _random_fill(self):
        """
        Randomly fills the map using the initial fill percentage.
        The top and bottom rows are kept filled as walls.
        """
        if self.seed:
            random.seed(self.seed)

        for r in range(self.rows):
            for c in range(self.cols):
                if r in [0, self.rows - 1]:
                    self.map[r][c] = 1
                else:
                    self.map[r][c] = 1 if random.random() < self.rand_fill_pct else 0

    def _smooth(self):
        """
        Runs one smoothing iteration on the map.
        Tiles with more than 4 filled neighbors are filled, and tiles with less than 2 filled neighbors are emptied.
        """
        newmap = [[self.map[r][c] for c in range(self.cols)] for r in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                if self._tile_neighbors(r, c) >= 5:
                    newmap[r][c] = 1
                elif self._tile_neighbors(r, c) <= 1:
                    newmap[r][c] = 0

        self.map = newmap

    def _tile_neighbors(self, r: int, c: int) -> int:
        """
        Returns the number of filled-in neighbors (8-neighborhood) for a given tile.

        Args:
            r (int): The row index of the tile.
            c (int): The column index of the tile.

        Returns:
            int: The number of filled-in neighbors.
        """
        count = 0
        for i in range(r - 1, r + 2):
            for j in range(c - 1, c + 2):
                if self._in_map(i, j):
                    if i != r or j != c:
                        count += self.map[i][j]
                elif i < 0 or i >= self.rows:
                    count += 1

        return count

    def _in_map(self, r: int, c: int) -> bool:
        """
        Checks if the given coordinates are within the map boundaries.

        Args:
            r (int): The row index.
            c (int): The column index.

        Returns:
            bool: True if the coordinates are within the map boundaries, False otherwise.
        """
        return r >= 0 and r < self.rows and c >= 0 and c < self.cols

    def get_map(self) -> List[List[int]]:
        """
        Returns the obstacle map.

        Returns:
            List[List[int]]: The obstacle map.
        """
        return self.map
