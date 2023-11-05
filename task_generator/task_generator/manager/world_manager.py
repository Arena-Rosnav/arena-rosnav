from typing import Collection, List, Optional, Tuple
import numpy as np
import random
import math
import scipy.signal

from task_generator.manager.utils import World, WorldEntities, WorldMap, WorldObstacleConfiguration, WorldOccupancy, WorldWalls, configurations_to_obstacles, occupancy_to_walls
from task_generator.shared import Position, Waypoint


class WorldManager:
    """
    The map manager manages the static map
    and is used to get new goal, robot and
    obstacle positions.
    """

    _world: World
    _walls: WorldWalls
    _forbidden_zones: List[Waypoint]

    _occupancy: np.ndarray

    def __init__(self, world_map: WorldMap, world_obstacles: Optional[Collection[WorldObstacleConfiguration]] = None):
        self.update_world(world_map=world_map, world_obstacles=world_obstacles)
        self.init_forbidden_zones()

    @property
    def world(self) -> World:
        return self._world

    @property
    def _origin(self) -> Position:
        return self.world.map.origin

    @property
    def _resolution(self) -> float:
        return self.world.map.resolution

    @property
    def walls(self) -> WorldWalls:
        return self._walls

    def update_map(self, map: WorldMap):
        # TODO deprecate in favor of direct call to update_world
        self.update_world(world_map=map)

    def update_world(
        self,
        world_map: WorldMap,
        world_obstacles: Optional[Collection[WorldObstacleConfiguration]] = None
    ):

        if world_obstacles is None:
            """this is OK because maps may not have preset entities"""
            world_obstacles = list()

        walls = occupancy_to_walls(
            occupancy_grid=world_map.occupancy.walls.grid,
            transform=lambda p: Position(p[0] * world_map.resolution + world_map.origin[0],
                                         (world_map.shape[0] - p[1]) * world_map.resolution + world_map.origin[1])
        )

        obstacles = configurations_to_obstacles(
            configurations=world_obstacles
        )

        entities = WorldEntities(
            obstacles=obstacles,
            walls=walls
        )

        self._world = World(
            entities=entities,
            map=world_map
        )

        for obstacle in self.world.entities.obstacles:
            self.world.map.occupancy.obstacles.occupy(
                Position(obstacle.position.x, obstacle.position.y), 1)

        self.init_forbidden_zones(
            [obstacle.position for obstacle in self.world.entities.obstacles])

    def init_forbidden_zones(self, init: Optional[List[Waypoint]] = None):
        if init is None:
            init = list()

        self._forbidden_zones = init

    def forbid(self, forbidden_zones: List[Waypoint]):
        self._forbidden_zones += forbidden_zones

        for zone in forbidden_zones:
            self.world.map.occupancy.forbidden.occupy(
                Position(zone.x, zone.y), 1)

    def get_random_pos_on_map(self, safe_dist: float, forbid: bool = True, forbidden_zones: Optional[List[Waypoint]] = None) -> Waypoint:
        """
        This function is used by the robot manager and
        obstacles manager to get new positions for both
        robot and obstalces.
        The function will choose a position at random
        and then validate the position. If the position
        is not valid a new position is chosen. When
        no valid position is found after 100 retries
        an error is thrown.
        Args:
            safe_dist: minimal distance to the next
                obstacles for calculated positons
            forbid: add returned waypoint to forbidden zones
            forbidden_zones: Array of (x, y, radius),
                describing circles on the map. New
                position should not lie on forbidden
                zones e.g. the circles.
                x, y and radius are in meters
        Returns:
            A tuple with three elements: x, y, theta
        """
        # safe_dist is in meters so at first calc safe dist to distance on
        # map -> resolution of map is m / cell -> safe_dist in cells is
        # safe_dist / resolution
        safe_dist_in_cells = math.ceil(
            safe_dist / self._resolution) + 1

        if forbidden_zones is None:
            forbidden_zones = []

        forbidden_zones_in_cells: List[Waypoint] = [
            Waypoint(
                math.ceil(point[0] / self._resolution),
                math.ceil(point[1] / self._resolution),
                math.ceil(point[2] / self._resolution)
            )
            for point in self._forbidden_zones + forbidden_zones
        ]

        # Now get index of all cells were dist is > safe_dist_in_cells
        possible_cells: List[Tuple[np.intp, np.intp]] = np.array(
            np.where(self._occupancy > safe_dist_in_cells)).transpose().tolist()

        # return (random.randint(1,6), random.randint(1, 9), 0)
        assert len(possible_cells) > 0, "No cells available"

        # The position should not lie in the forbidden zones and keep the safe
        # dist to these zones as well. We could remove all cells here but since
        # we only need one position and the amount of cells can get very high
        # we just pick positions at random and check if the distance to all
        # forbidden zones is high enough

        while len(possible_cells) > 0:

            # Select a random cell
            x, y = possible_cells.pop(random.randrange(len(possible_cells)))

            # Check if valid
            if self._is_pos_valid(float(x), float(y), safe_dist_in_cells, forbidden_zones_in_cells):
                break

        else:
            raise Exception("can't find any non-occupied spaces")

        theta = random.uniform(-math.pi, math.pi)

        point: Waypoint = Waypoint(
            float(
                np.round(y * self._resolution + self._origin[0], 3)),
            float(
                np.round(x * self._resolution + self._origin[1], 3)),
            theta
        )

        if forbid:
            self._forbidden_zones.append(point)

        return point

    def _is_pos_valid(self, x: float, y: float, safe_dist: float, forbidden_zones: List[Waypoint]):
        """
        @safe_dist: minimal distance to the next obstacles for calculated positions
        """
        for p in forbidden_zones:
            f_x, f_y, radius = p

            # euklidian distance to the forbidden zone
            dist = math.floor(np.linalg.norm(
                np.array([x, y]) - np.array([f_x, f_y]))) - radius

            if dist <= safe_dist:
                return False

        return True

    def get_available_positions(self, safe_dist: float) -> Collection[np.ndarray]:
        filt = np.full((int(2*safe_dist + 1), int(2*safe_dist + 1)), 1)
        spread = scipy.signal.convolve2d(
            self._occupancy,
            filt
        )
        return np.where(spread == WorldOccupancy.EMPTY)

    def get_random_available_position(self, safe_dist: float):
        return np.random.choice(list(self.get_available_positions(safe_dist=safe_dist)))
