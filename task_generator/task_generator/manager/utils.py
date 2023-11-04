"""
    This file exists to make world_manager more readable
"""

import dataclasses
import numpy as np
from typing import Any, Callable, Collection, Optional, Tuple, Dict, List, Set, Type

from map_distance_server.srv import GetDistanceMapResponse
from task_generator.shared import Obstacle, Position, PositionOrientation

from genpy.rostime import Time
from geometry_msgs.msg import Pose

# TYPES

@dataclasses.dataclass
class WorldMap:
    occupancy: np.ndarray
    origin: Position
    resolution: float
    time: Time

    @classmethod
    def from_distmap(cls: Type["WorldMap"], distmap: GetDistanceMapResponse) -> "WorldMap":
        return cls(
            occupancy = np.array(distmap.data).reshape((distmap.info.height, distmap.info.width)),
            origin = (distmap.info.origin.position.x, distmap.info.origin.position.y),
            resolution = distmap.info.resolution,
            time = distmap.info.map_load_time
        )
    
    
WorldWall = Tuple[Position, Position]
WorldWalls = Set[WorldWall]


@dataclasses.dataclass
class WorldEntityConfiguration:
    """
    only use this for receiving ros messages
    """
    position: PositionOrientation
    scaling: float
    model_name: str
    extra: Optional[Dict] = None #TODO pass this through to simulator

WorldEntities = Collection[Obstacle]

@dataclasses.dataclass
class World:
    entities: WorldEntities
    map: WorldMap
    walls: WorldWalls
    extras: Optional[Dict[str, Any]] = dataclasses.field(default_factory=lambda:dict())

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
            return set([((start, major), (end, major)) for major, segment in self.items() for start, end in segment])

        else:
            return set([((major, start), (major, end)) for major, segment in self.items() for start, end in segment])


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

    return set([(transform(wall[0]), transform(wall[1])) for wall in walls])