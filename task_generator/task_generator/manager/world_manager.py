from typing import Collection, List, Optional, Tuple
import numpy as np
import scipy.signal

from task_generator.manager.utils import World, WorldEntities, WorldMap, WorldObstacleConfiguration, WorldOccupancy, WorldWalls, configurations_to_obstacles, occupancy_to_walls
from task_generator.shared import Position, PositionRadius


class WorldManager:
    """
    The map manager manages the static map
    and is used to get new goal, robot and
    obstacle positions.
    """

    _world: World
    _classic_forbidden_zones: List[PositionRadius]

    def __init__(self, world_map: WorldMap, world_obstacles: Optional[Collection[WorldObstacleConfiguration]] = None):
        self._classic_forbidden_zones = []
        self.update_world(world_map=world_map, world_obstacles=world_obstacles)

    @property
    def world(self) -> World:
        return self._world
    
    @property
    def _shape(self) -> Tuple[int, int]:
        return self._world.map.shape[0], self._world.map.shape[1]

    @property
    def _origin(self) -> Position:
        return self._world.map.origin

    @property
    def _resolution(self) -> float:
        return self._world.map.resolution

    @property
    def walls(self) -> WorldWalls:
        return self._world.entities.walls

    def update_world(
        self,
        world_map: WorldMap,
        world_obstacles: Optional[Collection[WorldObstacleConfiguration]] = None
    ):

        if world_obstacles is None:
            """this is OK because maps may not have preset entities"""
            world_obstacles = list()

        walls = occupancy_to_walls(
            occupancy_grid=world_map.occupancy._walls.grid,
            transform=world_map.tf_grid2pos
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
            self.world.map.occupancy.obstacle_occupy(Position(obstacle.position.x, obstacle.position.y), 1)


    def forbid(self, forbidden_zones: List[Position]):
        for zone in forbidden_zones:
            self.world.map.occupancy.forbidden_occupy(zone, 1)

    def forbid_clear(self):
        self._world.map.occupancy.forbidden_clear()

    def _classic_get_random_pos_on_map(self, safe_dist: float, forbid: bool = True, forbidden_zones: Optional[List[PositionRadius]] = None) -> Position:
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

        import math
        import random

        def is_pos_valid(x: float, y: float, safe_dist: float, forbidden_zones: List[PositionRadius]):
            """
            @safe_dist: minimal distance to the next obstacles for calculated positions
            """
            for p in forbidden_zones:
                # euklidian distance to the forbidden zone
                dist = math.floor(np.linalg.norm(
                    np.array([x, y]) - np.array([p.x, p.y]))) - p.radius

                if dist <= safe_dist:
                    return False

            return True

        safe_dist_in_cells = math.ceil(
            safe_dist / self.world.map.resolution) + 1

        forbidden_zones_in_cells: List[PositionRadius] = [
            PositionRadius(
                math.ceil(point[0] / self.world.map.resolution),
                math.ceil(point[1] / self.world.map.resolution),
                math.ceil(point[2] / self.world.map.resolution)
            )
            for point in self._classic_forbidden_zones + (forbidden_zones if forbidden_zones is not None else [])
        ]

        # Now get index of all cells were dist is > safe_dist_in_cells
        possible_cells: List[Tuple[np.intp, np.intp]] = np.array(
            np.where(self.world.map.occupancy.grid > safe_dist_in_cells)).transpose().tolist()

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
            if is_pos_valid(float(x), float(y), safe_dist_in_cells, forbidden_zones_in_cells):
                break

        else:
            raise Exception("can't find any non-occupied spaces")

        point = PositionRadius(
            float(np.round(x * self.world.map.resolution + self.world.map.origin.x, 3)),
            float(np.round(y * self.world.map.resolution + self.world.map.origin.y, 3)),
            safe_dist
        )

        if forbid:
            self._classic_forbidden_zones.append(point)

        return Position(point.x, point.y)

    def get_positions_on_map(self, n: int, safe_dist: float, forbidden_zones: Optional[List[PositionRadius]] = None, forbid: bool = True) -> List[Position]:
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
        """
        # safe_dist is in meters so at first calc safe dist to distance on
        # map -> resolution of map is m / cell -> safe_dist in cells is
        # safe_dist / resolution

        if n < 0: #TODO profile when this is faster
            return [self._classic_get_random_pos_on_map(safe_dist = safe_dist, forbidden_zones=forbidden_zones) for _ in range(n)]

        max_depth = 10

        if forbidden_zones is None:
            forbidden_zones = []

        fork = self._world.map.occupancy.fork()

        for zone in forbidden_zones:
            fork.occupy(Position(zone.x, zone.y), zone.radius / self._resolution)

        available_positions = self._occupancy_to_available(occupancy=fork.grid, safe_dist=safe_dist / self._resolution)

        banned: np.ndarray = np.array([[]])

        min_dist: float = safe_dist / self._resolution

        def sample(target: int) -> Collection[Position]:

            result: List[Position] = list()
            depth: int = 0

            to_produce = target

            while depth < max_depth:

                candidates = available_positions[np.random.choice(len(available_positions), to_produce, replace=False), :]

                for candidate in candidates:
                    if banned.size and np.any(np.linalg.norm(banned.T - candidate, axis=0) <= min_dist):
                        continue;

                    np.append(banned, candidate)
                    result.append(self._world.map.tf_grid2pos((candidate[0], candidate[1])))

                to_produce = target - len(result)
                if to_produce <= 0:
                    break;

            else:
                raise RuntimeError(f"Failed to find free position after {depth} tries")

            return result

        points = sample(n)

        if forbid:
            fork.commit()

        return list(points)
    
    def get_position_on_map(self, safe_dist: float, forbidden_zones: Optional[List[PositionRadius]] = None, forbid: bool = True) -> Position:
        return self.get_positions_on_map(n=1, safe_dist=safe_dist, forbidden_zones=forbidden_zones)[0]

    def _occupancy_to_available(self, occupancy: np.ndarray, safe_dist: float) -> np.ndarray:

        filt_size = int(2*safe_dist + 1)
        filt = np.full((filt_size, filt_size), 1) / (filt_size ** 2)

        spread = scipy.signal.convolve2d(
            WorldOccupancy.not_full(occupancy).astype(np.uint8) * np.iinfo(np.uint8).max,
            filt,
            mode="full",
            boundary="fill",
            fillvalue=int(WorldOccupancy.FULL)
        )

        return np.transpose(np.where(WorldOccupancy.empty(spread)))

    