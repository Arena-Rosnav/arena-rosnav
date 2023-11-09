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

    def __init__(self, world_map: WorldMap, world_obstacles: Optional[Collection[WorldObstacleConfiguration]] = None):
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
        Returns:
            A tuple with three elements: x, y, theta
        """
        # safe_dist is in meters so at first calc safe dist to distance on
        # map -> resolution of map is m / cell -> safe_dist in cells is
        # safe_dist / resolution

        max_depth = 10

        if forbidden_zones is None:
            forbidden_zones = []

        fork = self._world.map.occupancy.fork()

        for zone in forbidden_zones:
            fork.occupy(Position(zone.x, zone.y), zone.radius / self._resolution)

        available_positions = self._occupancy_to_available(occupancy=fork.grid, safe_dist=safe_dist / self._resolution)

        if not len(available_positions):
            return [Position(x=0, y=0) for i in range(n)]

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

        np.save("/home/vova/occupancy.npz", occupancy)
        np.save("/home/vova/filter.npz", filt)

        spread = scipy.signal.convolve2d(
            WorldOccupancy.is_empty(occupancy).astype(np.float64),
            filt,
            mode="full",
            boundary="fill",
            fillvalue=False
        )

        np.save("/home/vova/spread.npz", spread)

        return np.transpose(np.where(spread))

    