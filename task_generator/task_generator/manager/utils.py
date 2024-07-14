"""
    This file exists to make world_manager more readable
"""

import dataclasses
import itertools
import os
import numpy as np
from typing import Callable, Collection, Optional, Tuple, Dict, List
import scipy.interpolate

from rospkg import RosPack

from task_generator.shared import Obstacle, Position, PositionOrientation, PositionRadius

from genpy.rostime import Time
from task_generator.utils import ModelLoader

import nav_msgs.srv as nav_srvs

# TYPES


_WorldWall = Tuple[Position, Position]
WorldWalls = Collection[_WorldWall]
WorldObstacles = Collection[Obstacle]


@dataclasses.dataclass
class WorldObstacleConfiguration:
    """
    only use this for receiving ros messages
    """
    position: PositionOrientation
    model_name: str
    extra: Dict


@dataclasses.dataclass
class WorldEntities:
    obstacles: WorldObstacles
    walls: WorldWalls


class WorldOccupancy:
    FULL: float = 0
    EMPTY: float = 100
    SCALE: float = 100

    _grid: np.ndarray

    def __init__(self, grid: np.ndarray):
        """
        Take a raw grid within (EMPTY, FULL) range and wrap it in a WorldOccupancy instance.

        Args:
            grid: grid to be wrapped

        Returns:
            WorldOccupancy
        """
        self._grid = grid

    @classmethod
    def from_map(cls, input_map: np.ndarray) -> "WorldOccupancy":
        """
        Take an input_map with any value range, linearly remap it to (EMPTY, FULL) range. Returns a WorldOccupancy instance.

        Args:
            input_map: map to be converted

        Returns:
            WorldOccupancy
        """
        remap = scipy.interpolate.interp1d(
            [100, 0],
            [cls.EMPTY, cls.FULL]
        )
        return cls(input_map)

    @classmethod
    def empty(cls, grid: np.ndarray) -> np.ndarray:
        """
        check where grid is empty (= grid ≈ EMPTY)

        Args:
            grid: grid to analyze

        Returns:
            np.ndarray
        """
        return np.isclose(grid, cls.EMPTY)
    
    @classmethod
    def not_empty(cls, grid: np.ndarray) -> np.ndarray:
        """
        check where grid is not empty (= invert(empty(grid)))

        Args:
            grid: grid to analyze

        Returns:
            np.ndarray
        """
        return np.invert(cls.empty(grid))

    @classmethod
    def emptyish(cls, grid: np.ndarray, thresh: Optional[float] = None) -> np.ndarray:
        """
        check where grid is empty-ish (= grid >= thresh)

        Args:
            grid: grid to analyze
            thresh: treshold value (default: mean(FULL, EMPTY))

        Returns:
            np.ndarray
        """
        return np.abs(grid - cls.EMPTY) < np.abs(grid - cls.FULL)

    @classmethod
    def full(cls, grid: np.ndarray) -> np.ndarray:
        """
        check where grid is full (= grid ≈ FULL)

        Args:
            grid: grid to analyze

        Returns:
            np.ndarray
        """
        return np.isclose(grid, cls.FULL)

    @classmethod
    def not_full(cls, grid: np.ndarray) -> np.ndarray:
        """
        check where grid is not full (= invert(full(grid)))

        Args:
            grid: grid to analyze

        Returns:
            np.ndarray
        """
        return np.invert(cls.full(grid))

    @classmethod
    def fullish(cls, grid: np.ndarray, thresh: Optional[float] = None) -> np.ndarray:
        """
        check where grid is not empty-ish (= invert(emptyish(grid)))

        Args:
            grid: grid to analyze
            thresh: treshold value (default: mean(FULL, EMPTY))

        Returns:
            np.ndarray
        """
        return np.invert(cls.emptyish(grid, thresh))

    @property
    def grid(self) -> np.ndarray:
        """
        get current grid (reference)

        Args:
            None

        Returns:
            np.ndarray
        """
        return self._grid

    def clear(self):
        """
        clear occupation

        Args:
            None

        Returns:
            None
        """
        self.grid.fill(self.EMPTY)

    def occupy(self, lo: Tuple[int, int], hi: Tuple[int, int], inv: bool = False):
        """
        occupy the rectangle spanned by (lo,hi)

        Args:
            lo: (min_x, min_y)
            hi: (max_x, max_y)
            inv: invert (occupy everything _except_ rectangle)

        Returns:
            None
        """

        ly, hy = np.clip(np.array([lo[1], hi[1]]), 0, self._grid.shape[0] - 1)
        lx, hx = np.clip(np.array([lo[0], hi[0]]), 0, self._grid.shape[1] - 1)

        if inv:
            self._grid[:int(ly),:] = self.FULL
            self._grid[int(hy):,:] = self.FULL
            self._grid[:,:int(lx)] = self.FULL
            self._grid[:,int(hx):] = self.FULL
        else:
            self._grid[
                int(ly):int(hy),
                int(lx):int(hx)
            ] = self.FULL


class WorldLayers:
    _walls: WorldOccupancy      # walls
    _obstacle: WorldOccupancy  # intrinsic obstcales
    _forbidden: WorldOccupancy  # task obstacles

    def __init__(self, walls: WorldOccupancy):
        self._walls = walls
        self._obstacle = WorldOccupancy(
            np.full(walls.grid.shape, WorldOccupancy.EMPTY))
        self._forbidden = WorldOccupancy(
            np.full(walls.grid.shape, WorldOccupancy.EMPTY))

        self._combined_cache = None

    _combined_cache: Optional[WorldOccupancy]

    def _invalidate_combined_cache(self):
        self._combined_cache = None

    @property
    def _combined(self) -> WorldOccupancy:
        if self._combined_cache is None:
            self._combined_cache = WorldOccupancy(np.minimum.reduce([
                self._walls.grid,
                self._obstacle.grid,
                self._forbidden.grid
            ]))

        return self._combined_cache

    @property
    def grid(self) -> np.ndarray:
        return self._combined.grid

    # obstacle interface
    def obstacle_occupy(self, lo: Tuple[int, int], hi: Tuple[int, int]):
        self._obstacle.occupy(lo, hi)
        self._combined.occupy(lo, hi)

    def obstacle_clear(self):
        self._obstacle.clear()
        self._invalidate_combined_cache()

    # forbidden interface
    def forbidden_occupy(self, lo: Tuple[int, int], hi: Tuple[int, int]):
        self._forbidden.occupy(lo, hi)
        self._combined.occupy(lo, hi)

    def forbidden_clear(self):
        self._forbidden.clear()
        self._invalidate_combined_cache()

    class WorldLayersFork:

        _base: "WorldLayers"
        _grid: WorldOccupancy

        def __init__(self, base: "WorldLayers"):
            self._base = base
            self._grid = WorldOccupancy(self._base.grid.copy())

        def commit(self):
            self._base._forbidden = self._grid
            self._base._invalidate_combined_cache()

        def occupy(self, lo:Tuple[int, int], hi: Tuple[int, int], **kwargs):
            self._grid.occupy(lo, hi, **kwargs)

        @property
        def grid(self):
            return self._grid.grid

    def fork(self):
        return WorldLayers.WorldLayersFork(self)


@dataclasses.dataclass
class WorldMap:
    occupancy: WorldLayers
    origin: Position
    resolution: float
    time: Time

    @staticmethod
    def from_distmap(distmap: nav_srvs.GetMapResponse) -> "WorldMap":
        return WorldMap(
            occupancy=WorldLayers(
                walls=WorldOccupancy.from_map(
                    np.array(distmap.map.data).reshape(
                        (distmap.map.info.height, distmap.map.info.width))
                )
            ),
            origin=Position(
                distmap.map.info.origin.position.y,
                distmap.map.info.origin.position.x
            ),
            resolution=distmap.map.info.resolution,
            time=distmap.map.info.map_load_time
        )

    @property
    def shape(self) -> Tuple[int, ...]:
        return self.occupancy._walls.grid.shape

    def tf_pos2grid(self, position: Position) -> Tuple[int, int]:
        return np.round((position.y - self.origin.y) / self.resolution), np.round((position.x - self.origin.x) / self.resolution)

    def tf_grid2pos(self, grid_pos: Tuple[int, int]) -> Position:
        return Position(x = grid_pos[1] * self.resolution + self.origin.y, y = grid_pos[0] * self.resolution + self.origin.x)

    def tf_posr2rect(self, posr: PositionRadius) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        lo = self.tf_pos2grid(Position(posr.x - posr.radius, posr.y - posr.radius))
        hi = self.tf_pos2grid(Position(posr.x + posr.radius, posr.y + posr.radius))
        return (lo, hi)


@dataclasses.dataclass
class World:
    entities: WorldEntities
    map: WorldMap

# END TYPES


def RLE_1D(grid: np.ndarray) -> List[List[int]]:
    """
    run-length encode walls in 1D (occupancy grid -> run_length[segments][rows])
    """
    res: List[List[int]] = list()
    for major in grid:
        run: int = 1
        last: int = major[0]
        subres: List[int] = [0]
        for minor in major[1:]:
            if minor == last:
                run += 1
            else:
                subres.append(run)
                run = 1
                last = minor
        subres.append(run)
        res.append(subres)
    return res


class _WallLines(Dict[float, List[Tuple[float, float]]]):
    """
    Helper class for efficiently merging collinear line segments
    """

    _inverted: bool

    def __init__(self, inverted: bool = False, *args, **kwargs):
        """
        inverted=True for y-axis pass
        """
        super().__init__(*args, **kwargs)
        self._inverted = inverted

    def add(self, major: float, minor: float, length: float = 1):
        """
        add a wall segment in row <major> at position <minor> with length <length> and merge with previous line segment if their endpoints touch
        """
        if major not in self:
            self[major] = [(minor, minor + length)]
            return

        last = self[major][-1]

        if minor == last[1]:
            self[major][-1] = (last[0], minor + length)
        else:
            self[major].append((minor, minor + length))

    @property
    def lines(self) -> WorldWalls:
        """
        get WorldWalls object
        """
        if not self._inverted:
            return set([(Position(start, major), Position(end, major)) for major, segment in self.items() for start, end in segment])
        else:
            return set([(Position(major, start), Position(major, end)) for major, segment in self.items() for start, end in segment])


def RLE_2D(grid: np.ndarray) -> WorldWalls:
    """
    rudimentary (but fast) 2D extension of 1D-RLE to 2D (occupancy grid -> WorldWalls)
    """

    walls_x = _WallLines()
    walls_y = _WallLines(inverted=True)

    for y, rles in enumerate(RLE_1D(grid)):
        distance: int = 0
        for run in rles:
            distance += run
            walls_x.add(distance, y)

    for x, rles in enumerate(RLE_1D(grid.T)):
        distance: int = 0
        for run in rles:
            distance += run
            walls_y.add(distance, x)

    return set().union(walls_x.lines, walls_y.lines)


def occupancy_to_walls(occupancy_grid: np.ndarray, transform: Optional[Callable[[Tuple[int, int]], Position]] = None) -> WorldWalls:
    walls = RLE_2D(grid=WorldOccupancy.not_full(occupancy_grid))

    if transform is None:
        transform = lambda p: Position(x=p[0], y=p[1])

    return [(transform(wall[0]), transform(wall[1])) for wall in walls]


_world_model_loader = ModelLoader(os.path.join(
    RosPack().get_path("arena_simulation_setup"), "tmp", "models"))


def configurations_to_obstacles(configurations: Collection[WorldObstacleConfiguration]) -> WorldObstacles:

    name_gen = itertools.count()

    return [Obstacle(
        position=configuration.position,
        name=f"world_obstacle_{next(name_gen)}",
        model=_world_model_loader.bind(configuration.model_name),
        extra=configuration.extra
    ) for configuration in configurations]
