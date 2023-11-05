"""
    This file exists to make world_manager more readable
"""

import dataclasses
import itertools
import os
import numpy as np
from typing import Callable, Collection, Optional, Tuple, Dict, List

from rospkg import RosPack

from map_distance_server.srv import GetDistanceMapResponse
from task_generator.shared import Obstacle, Position, PositionOrientation

from genpy.rostime import Time
from task_generator.utils import ModelLoader

# TYPES


WorldWall = Tuple[Position, Position]
WorldWalls = Collection[WorldWall]
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
    FULL: np.uint8 = np.uint8(np.iinfo(np.uint8).min)
    EMPTY: np.uint8 = np.uint8(np.iinfo(np.uint8).max)

    _grid: np.ndarray

    @property
    def grid(self) -> np.ndarray:
        return self._grid

    def __init__(self, grid: np.ndarray):
        self._grid = grid

    def clear(self):
        self.grid.fill(WorldOccupancy.FULL)

    def occupy(self, zone: Position, radius: float):
        self._grid[
            int(zone[1]-radius):int(zone[1]+radius),
            int(zone[0]-radius):int(zone[0]+radius)
        ] = WorldOccupancy.FULL


class WorldLayers:
    walls: WorldOccupancy      # walls
    obstacles: WorldOccupancy  # intrinsic obstcales
    forbidden: WorldOccupancy  # task obstacles

    def __init__(self, walls: WorldOccupancy):
        self.walls = walls
        self.obstacles = WorldOccupancy(
            np.full(walls.grid.shape, WorldOccupancy.EMPTY))
        self.forbidden = WorldOccupancy(
            np.full(walls.grid.shape, WorldOccupancy.EMPTY))

    @property
    def combined_occupancy(self) -> WorldOccupancy:
        return np.minimum.reduce([
            self.walls.grid,
            self.obstacles.grid,
            self.forbidden.grid
        ])


@dataclasses.dataclass
class WorldMap:
    occupancy: WorldLayers
    origin: Position
    resolution: float
    time: Time

    @staticmethod
    def from_distmap(distmap: GetDistanceMapResponse) -> "WorldMap":
        return WorldMap(
            occupancy=WorldLayers(
                walls=WorldOccupancy(
                    np.array(distmap.data).reshape((distmap.info.height, distmap.info.width))
                )
            ),
            origin=Position(
                distmap.info.origin.position.x,
                distmap.info.origin.position.y
            ),
            resolution=distmap.info.resolution,
            time=distmap.info.map_load_time
        )

    @property
    def shape(self) -> Tuple[int, ...]:
        return self.occupancy.walls.grid.shape


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
            self[major] = [(minor, minor+length)]
            return

        last = self[major][-1]

        if minor == last[1]:
            self[major][-1] = (last[0], minor+length)
        else:
            self[major].append((minor, minor+length))

    @property
    def lines(self) -> WorldWalls:
        """
        get WorldWalls object
        """
        if self._inverted:
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


def occupancy_to_walls(occupancy_grid: np.ndarray, transform: Optional[Callable[[Position], Position]] = None) -> WorldWalls:
    binary_grid = (occupancy_grid > occupancy_grid.mean()).astype(np.bool_)
    walls = RLE_2D(grid=binary_grid)

    if transform is None:
        transform = lambda p: p

    return [(transform(wall[0]), transform(wall[1])) for wall in walls]


_world_model_loader = ModelLoader(os.path.join(
    RosPack().get_path("arena-simulation-setup"), "tmp", "models"))


def configurations_to_obstacles(configurations: Collection[WorldObstacleConfiguration]) -> WorldObstacles:

    name_gen = itertools.count()

    return [Obstacle(
        position=configuration.position,
        name=f"world_obstacle_{next(name_gen)}",
        model=_world_model_loader.bind(configuration.model_name),
        extra=configuration.extra
    ) for configuration in configurations]
