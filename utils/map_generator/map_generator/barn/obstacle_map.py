from typing import List

import random


# class to generate occupancy grids using cellular automaton
class ObstacleMap:
    # rand_fill_pct is the initial fill percent
    # seed is the random seed
    # smooth_iter is the number of smoothing iterations to run
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

    # fill in map and run smoothing iterations
    def generate_map(self, obs_map: List[List[int]] = None) -> List[List[int]]:
        if obs_map:
            assert len(obs_map) in [self.rows, self.cols]
            assert len(obs_map[0]) in [self.rows, self.cols]
            self.map = obs_map
        else:
            self._random_fill()

        for _ in range(self.smooth_iter):
            self._smooth()

        return self.map

    # randomly fill in using the initial fill percent
    # keep top and bottom rows filled in as walls
    def _random_fill(self):
        if self.seed:
            random.seed(self.seed)

        for r in range(self.rows):
            for c in range(self.cols):
                # fill in top and bottom rows
                if r in [0, self.rows - 1]:
                    self.map[r][c] = 1
                else:
                    # fill in the cell if random.random() returns a value less than initial fill percent
                    self.map[r][c] = 1 if random.random() < self.rand_fill_pct else 0

    # run one smoothing iteration with fill threshold of 5 and clear threshold of 1
    def _smooth(self):
        # use buffer map so that evolutions within same iteration do not affect each other
        newmap = [[self.map[r][c] for c in range(self.cols)] for r in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                # if more than 4 filled neighbors, fill this tile
                if self._tile_neighbors(r, c) >= 5:
                    newmap[r][c] = 1

                # if less than 2 filled neighbors, empty this one
                elif self._tile_neighbors(r, c) <= 1:
                    newmap[r][c] = 0

        self.map = newmap

    # returns number of filled-in neighbors (neighborhood of 8)
    def _tile_neighbors(self, r: int, c: int) -> int:
        count = 0
        for i in range(r - 1, r + 2):
            for j in range(c - 1, c + 2):
                if self._in_map(i, j):
                    if i != r or j != c:
                        count += self.map[i][j]

                # if on the top or bottom, add to wall neighbors
                elif i < 0 or i >= self.rows:
                    count += 1

        return count

    def _in_map(self, r: int, c: int) -> bool:
        return r >= 0 and r < self.rows and c >= 0 and c < self.cols

    def get_map(self):
        return self.map
