"""
    This file exists to make world_manager more readable
"""

import dataclasses
import itertools
import os
from typing import Callable, Collection, Dict, List, Optional, Tuple

import nav_msgs.msg
import numpy as np
import scipy.interpolate

import task_generator.utils.arena as Utils
from task_generator.shared import (Obstacle, Position, PositionOrientation,
                                   PositionRadius, Wall)
from task_generator.utils import ModelLoader
from task_generator.utils.time import Time

# TYPES


WorldWalls = Collection[Wall]
WorldObstacles = Collection[Obstacle]


@dataclasses.dataclass
class WorldObstacleConfiguration:
    """
    only use this for receiving ros messages
    """
    position: PositionOrientation
    model_name: str
    extra: Dict

    @classmethod
    def parse(cls, obj: dict) -> "WorldObstacleConfiguration":

        x = obj['position'][0]
        y = obj['position'][1]
        θ = obj['position'][2]

        return cls(
            position=PositionOrientation(
                x=x,
                y=y,
                orientation=θ,
            ),
            model_name=obj['model'],
            extra=obj,
        )


WorldObstacleConfigurations = Collection[WorldObstacleConfiguration]


@dataclasses.dataclass
class WorldEntities:
    obstacles: WorldObstacles
    walls: WorldWalls


class WorldOccupancy:
    FULL: np.uint8 = np.uint8(np.iinfo(np.uint8).min)
    EMPTY: np.uint8 = np.uint8(np.iinfo(np.uint8).max)

    _grid: np.ndarray

    def __init__(self, grid: np.ndarray):
        self._grid = grid

    @staticmethod
    def from_map(input_map: np.ndarray) -> "WorldOccupancy":
        remap = scipy.interpolate.interp1d([input_map.max(), input_map.min()], [
                                           WorldOccupancy.EMPTY, WorldOccupancy.FULL])
        return WorldOccupancy(remap(input_map))

    @staticmethod
    def empty(grid: np.ndarray) -> np.ndarray:
        return np.isclose(grid, WorldOccupancy.EMPTY)

    @staticmethod
    def not_empty(grid: np.ndarray) -> np.ndarray:
        return np.invert(WorldOccupancy.empty(grid))

    @staticmethod
    def emptyish(grid: np.ndarray,
                 thresh: Optional[float] = None) -> np.ndarray:
        if thresh is None:
            thresh = float((int(WorldOccupancy.FULL) +
                           int(WorldOccupancy.EMPTY)) / 2)
        return grid >= thresh

    @staticmethod
    def full(grid: np.ndarray) -> np.ndarray:
        return np.isclose(grid, WorldOccupancy.FULL)

    @staticmethod
    def not_full(grid: np.ndarray) -> np.ndarray:
        return np.invert(WorldOccupancy.full(grid))

    @staticmethod
    def fullish(grid: np.ndarray,
                thresh: Optional[float] = None) -> np.ndarray:
        return np.invert(WorldOccupancy.emptyish(grid, thresh))

    @property
    def grid(self) -> np.ndarray:
        return self._grid

    def clear(self):
        self.grid.fill(WorldOccupancy.EMPTY)

    def occupy(self, lo: Tuple[int, int], hi: Tuple[int, int]):
        ly, hy = np.clip(np.array([lo[1], hi[1]]), 0, self._grid.shape[0] - 1)
        lx, hx = np.clip(np.array([lo[0], hi[0]]), 0, self._grid.shape[1] - 1)
        self._grid[
            int(ly):int(hy),
            int(lx):int(hx)
        ] = WorldOccupancy.FULL


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

        def occupy(self, lo: Tuple[int, int], hi: Tuple[int, int]):
            self._grid.occupy(lo, hi)

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
    def from_costmap(
            occupancy_grid: nav_msgs.msg.OccupancyGrid) -> "WorldMap":
        # Convert occupancy grid data to numpy array
        grid_data = np.array(
            occupancy_grid.data).reshape(
            (occupancy_grid.info.height,
             occupancy_grid.info.width))

        # Normalize data to 0-255 range (0 = occupied, 255 = free)
        normalized_data = np.interp(
            grid_data, (100, 0), (WorldOccupancy.EMPTY, WorldOccupancy.FULL)).astype(
            np.uint8)

        return WorldMap(
            occupancy=WorldLayers(
                walls=WorldOccupancy(normalized_data)
            ),
            origin=Position(
                x=occupancy_grid.info.origin.position.y,
                y=occupancy_grid.info.origin.position.x
            ),
            resolution=occupancy_grid.info.resolution,
            time=Time.from_time(occupancy_grid.info.map_load_time)
        )

    @property
    def shape(self) -> Tuple[int, ...]:
        return self.occupancy._walls.grid.shape

    def tf_pos2grid(self, position: Position) -> Tuple[int, int]:
        return np.round((position.y - self.origin.y) / self.resolution), np.round(
            self.shape[1] - (position.x - self.origin.x) / self.resolution)

    def tf_grid2pos(self, grid_pos: Tuple[float, float]) -> Position:
        return Position(
            x=grid_pos[1] * self.resolution + self.origin.y,
            y=(grid_pos[0]) * self.resolution + self.origin.x
        )

    def tf_posr2rect(
            self, posr: PositionRadius) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        lo = self.tf_pos2grid(
            Position(
                x=posr.x - posr.radius,
                y=posr.y - posr.radius,
            )
        )
        hi = self.tf_pos2grid(
            Position(
                x=posr.x + posr.radius,
                y=posr.y + posr.radius,
            )
        )
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
            return set([
                Wall(
                    Start=Position(x=start, y=major),
                    End=Position(x=end, y=major)
                )
                for major, segment in self.items()
                for start, end in segment]
            )
        else:
            return set([
                Wall(
                    Start=Position(x=major, y=start),
                    End=Position(x=major, y=end)
                )
                for major, segment in self.items()
                for start, end in segment]
            )


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


def occupancy_to_walls(
    occupancy_grid: np.ndarray,
    transform: Optional[Callable[[Tuple[float, float]], Position]] = None
) -> WorldWalls:

    walls = RLE_2D(grid=WorldOccupancy.not_full(occupancy_grid))

    if transform is None:
        def _transform(p):
            return Position(x=p[0], y=p[1])
        transform = _transform

    return [
        Wall(
            Start=transform((wall.Start.x, wall.Start.y)),
            End=transform((wall.End.x, wall.End.y)),
        )
        for wall in walls
    ]


_obstacles_loader = ModelLoader(
    os.path.join(
        Utils.get_simulation_setup_path(),
        'entities',
        'obstacles',
        'static')
)


def configurations_to_obstacles(
    configurations: Collection[WorldObstacleConfiguration]
) -> WorldObstacles:

    name_gen = itertools.count()

    return [Obstacle(
        position=configuration.position,
        name=f"world_obstacle_{next(name_gen)}",
        model=_obstacles_loader.bind(configuration.model_name),
        extra=configuration.extra
    ) for configuration in configurations]
